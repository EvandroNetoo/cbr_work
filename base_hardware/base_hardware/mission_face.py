"""Mapeamento do snapshot de missão para as expressões disponíveis no OLED."""

from .face_animation import ExpressoesFaciais


def validate_expression(parameter_name: str, expression: str) -> str:
    if expression not in ExpressoesFaciais.EXPRESSOES:
        available = ', '.join(ExpressoesFaciais.EXPRESSOES)
        raise ValueError(f'{parameter_name}: expressão {expression!r} inválida. Use: {available}.')
    return expression


OPERATION_EXPRESSIONS = {
    'navigate': 'correndo',
    'finish': 'correndo',  # O último passo ainda navega até a posição final.
    'follow_wall': 'correndo',
    'analyze_scene': 'surpreso',
    'pick': 'bravo',
    'store': 'piscando',
    'retrieve': 'bravo',
    'place_on_table': 'feliz',
    'place_in_container': 'feliz',
    'place_on_shelf': 'feliz',
    'place_on_precision_table': 'bravo',
    'stack': 'bravo',
    'organize_precision': 'bravo',
}


def expression_for_mission(
    active_operation: str, state_known: bool, *, operation_expressions=None,
    idle='neutro', unknown_state='triste', unknown_operation='neutro',
) -> str:
    # Um inventário incerto tem prioridade sobre a operação ainda registrada.
    if not state_known:
        return unknown_state
    if not active_operation:
        return idle
    mapping = OPERATION_EXPRESSIONS if operation_expressions is None else operation_expressions
    return mapping.get(active_operation, unknown_operation)
