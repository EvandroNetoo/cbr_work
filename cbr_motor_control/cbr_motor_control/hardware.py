"""Construction and lifetime management for the copied Mariola drivers."""

from dataclasses import dataclass
from time import sleep

from .controle_motores import (
    ControleMotores,
    GrupoMotoresBrick,
    GrupoMotoresExpansao,
)
from .motores import Motores
from .placa_controle_motor import PlacaControleMotor
from .portas import Portas


@dataclass(frozen=True)
class HardwareSettings:
    expansion_port: int = Portas.SERIAL3
    baud_rate: int = 250000
    serial_timeout: float = 0.005
    front_left_id: int = 0
    front_right_id: int = 7
    front_left_inverted: bool = True
    front_right_inverted: bool = False
    rear_left_inverted: bool = False
    rear_right_inverted: bool = True
    configure_on_start: bool = True
    calibration_clockwise: int = 80
    calibration_counterclockwise: int = -80
    brake_kp: float = 3.0
    brake_kd: float = 10.0
    brake_delta: int = 20
    motor_kp: float = 2.0
    motor_ki: float = 2.0
    motor_kd: float = 2.0


class SimulatedController:
    """Drop-in controller used to inspect ROS commands without serial I/O."""

    def __init__(self):
        self._speeds = {
            'dianteiro_esquerdo': 0,
            'dianteiro_direito': 0,
            'traseiro_esquerdo': 0,
            'traseiro_direito': 0,
        }
        self._closed = False

    @property
    def velocidades_atuais(self):
        return dict(self._speeds)

    def definir_velocidades(self, **speeds):
        if self._closed:
            raise RuntimeError('O controle simulado já foi fechado')
        self._speeds.update(speeds)

    def frear(self):
        self.definir_velocidades(**dict.fromkeys(self._speeds, 0))

    def fechar(self):
        if not self._closed:
            self.frear()
            self._closed = True


class MariolaHardware:
    """Owns the controller and both serial connections."""

    def __init__(self, settings, logger):
        self._brick = None
        self._expansion_serial = None
        self._controller = None
        self._closed = False

        try:
            # Preserve the supplied main.py order: open the expansion bus and
            # create its motor objects before initializing the brick. The
            # brick handshake also gives the expansion boards time to settle.
            self._expansion_serial = Portas().abre_porta_serial(
                settings.expansion_port,
                settings.baud_rate,
                timeout=settings.serial_timeout,
            )
            if self._expansion_serial is None:
                raise RuntimeError(
                    'Não foi possível abrir o barramento dos motores dianteiros')

            front_left = PlacaControleMotor(
                self._expansion_serial, settings.front_left_id)
            front_right = PlacaControleMotor(
                self._expansion_serial, settings.front_right_id)
            self._brick = Motores(True)

            if settings.configure_on_start:
                self._configure(
                    self._brick, front_left, front_right, settings, logger)

            self._controller = ControleMotores([
                GrupoMotoresBrick(
                    self._brick,
                    nomes=('traseiro_esquerdo', 'traseiro_direito'),
                    invertidos=(
                        settings.rear_left_inverted,
                        settings.rear_right_inverted,
                    ),
                ),
                GrupoMotoresExpansao(
                    {
                        'dianteiro_esquerdo': front_left,
                        'dianteiro_direito': front_right,
                    },
                    invertidos={
                        'dianteiro_esquerdo':
                            settings.front_left_inverted,
                        'dianteiro_direito':
                            settings.front_right_inverted,
                    },
                ),
            ])
        except Exception:
            self._close_serials()
            raise

    @staticmethod
    def _report_response(description, result, logger):
        if result is None or result is False:
            # The original main.py prints/ignores these return values and
            # continues. A reset may also take effect before its reply is read.
            logger.warning(
                f'Sem confirmação de {description}; continuando como main.py')
        else:
            logger.info(f'Configuração confirmada: {description}')
        return result

    def _configure(
        self, brick, front_left, front_right, settings, logger
    ):
        """Run the setup sequence found in the supplied ``main.py``."""
        self._report_response(
            'reset dianteiro esquerdo', front_left.reset(), logger)
        self._report_response(
            'reset dianteiro direito', front_right.reset(), logger)
        calibration = brick.obtem_calibracao_motores()
        logger.info(f'Calibração dos motores traseiros: {calibration}')

        sleep(0.5)
        brick.set_modo_freio(Motores.HOLD)
        for name, motor in (
            ('dianteiro esquerdo', front_left),
            ('dianteiro direito', front_right),
        ):
            self._report_response(
                f'freio {name}',
                motor.set_freio(PlacaControleMotor.FREIO_TRAVADO),
                logger,
            )
            self._report_response(
                f'Kp do freio {name}',
                motor.set_kp_freio(settings.brake_kp),
                logger,
            )
            self._report_response(
                f'Kd do freio {name}',
                motor.set_kd_freio(settings.brake_kd),
                logger,
            )
            self._report_response(
                f'delta do freio {name}',
                motor.set_delta_freio(settings.brake_delta),
                logger,
            )

        # O movimento usa o modo PID, portanto os ganhos configurados aqui
        # controlam diretamente o caminho de velocidade das placas dianteiras.
        for name, motor in (
                    ('dianteiro esquerdo', front_left),
                    ('dianteiro direito', front_right),
                ):
            self._report_response(
                f'PID do motor {name}',
                motor.pid_motor(
                    settings.motor_kp, settings.motor_ki, settings.motor_kd),
                logger,
            )
            
        for name, motor in (
            ('dianteiro esquerdo', front_left),
            ('dianteiro direito', front_right),
        ):
            self._report_response(
                f'calibração manual {name}',
                motor.calibracao_manual(
                    settings.calibration_clockwise,
                    settings.calibration_counterclockwise,
                ),
                logger,
            )

    @property
    def velocidades_atuais(self):
        return self._controller.velocidades_atuais

    def definir_velocidades(self, **speeds):
        self._controller.definir_velocidades(**speeds)

    def frear(self):
        self._controller.frear()

    def fechar(self):
        if self._closed:
            return
        self._closed = True
        error = None
        if self._controller is not None:
            try:
                self._controller.fechar()
            except Exception as exc:
                error = exc
        self._close_serials()
        if error is not None:
            raise error

    def _close_serials(self):
        if self._brick is not None:
            try:
                self._brick.fechar()
            except Exception:
                pass
        if self._expansion_serial is not None:
            try:
                self._expansion_serial.close()
            except Exception:
                pass
