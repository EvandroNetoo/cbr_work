# Bancada de testes CBR

Painel local em português para registrar manualmente missões completas por
**missão + seed**, calcular a pontuação e acompanhar os resultados. Não importa
ROS no processo da interface, não movimenta o robô e funciona sem internet depois
da instalação. A geração opcional de planos chama o ROS 2 instalado por CLI.

## Iniciar

Neste workspace as dependências já foram instaladas no ambiente virtual da raiz:

```bash
cd ~/ros2_ws
bash src/cbr_work/tools/robot_test_dashboard/run.sh
```

Abra **http://127.0.0.1:8501**. Encerre com `Ctrl+C` no terminal.
O servidor escuta somente o notebook. Não é necessário executar `colcon build`
nem iniciar o robô.

Para instalar em outro notebook, use Python 3.12 e um ambiente próprio:

```bash
cd ~/ros2_ws/src/cbr_work/tools/robot_test_dashboard
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
bash run.sh
```

O script procura primeiro `.venv` do painel, depois `.venv` do workspace.
Também aceita `ROBOT_TEST_PYTHON=/caminho/absoluto/python`. As versões de pandas
e PyArrow foram fixadas para funcionar com o NumPy 1.x do workspace ROS.

## Registrar um teste

### Gerar cenário e plano por seed

Em **Novo teste → Gerar por seed**, selecione a prova e informe a seed. Em
**Caminhos e opções de planejamento**, confira a pasta e o Python do Commander,
o setup ROS do workspace, arena, mapa e parâmetros do planner. Os caminhos deste
workspace já vêm preenchidos. A metaarena e os decoys podem ficar vazios para
usar os padrões do Commander.

Todos os campos são salvos automaticamente no SQLite local ao confirmar a edição
(Enter ou sair do campo). Ao voltar à tela ou reiniciar o painel, os valores
continuam preenchidos, inclusive a seed. Não é necessário clicar em gerar para
salvar as preferências.

Clique em **Gerar cenário e plano**. O Commander gera os YAMLs de equipe e juiz;
depois o `mission_manager generate_plan` calcula as distâncias e otimiza o plano.
A geração usa processos em segundo plano e mostra andamento e logs. Requer
Commander instalado, workspace ROS compilado e máscara keepout junto ao mapa.
O planner local inicia automaticamente e não envia comandos de movimento.

Cada execução mantém seus arquivos e logs em `data/generations/<execução>/`
(ao lado do banco configurado por `ROBOT_TEST_DB`). Consulte execuções anteriores
em **Gerações salvas** e baixe os YAMLs. Falhas preservam os arquivos já gerados
e os logs. Fechar a aba não interrompe a geração. O limite de busca configurado
vale após as distâncias; há um limite adicional de 30 minutos para essa etapa e
inicialização. As preferências e essas pastas não fazem parte da exportação JSON
de fichas; os cenários e planos anexados às fichas fazem.

Se o otimizador atingir o limite de tempo ou de estados, consulte o log, aumente
**Limite da busca** ou **Máximo de estados da busca** e gere novamente. Um plano
parcial não é publicado como resultado concluído.

Para o plano, use **Salvar plano na pasta de missões**: o painel grava diretamente
em `src/cbr_work/mission_manager/config/plans`, sem depender da pasta Downloads do
navegador. Na configuração padrão, também grava no pacote instalado em
`install/mission_manager/share/mission_manager/config/plans`, se o pacote já está
instalado, disponibilizando o plano ao executor sem recompilar. O nome do arquivo
e o `plan_id` seguem o tipo da missão, substituindo o plano anterior:

| Prova | Arquivo |
|---|---|
| BMT | `1_basic_manipulation_test.yaml` |
| BTT1 | `2_basic_transportation_test_i.yaml` |
| BTT2 | `3_basic_transportation_test_ii.yaml` |
| AMT | `4_advanced_transportation_test.yaml` |
| ATT1 | `5_advanced_transportation_test_i.yaml` |
| ATT2 | `6_advanced_transportation_test_ii.yaml` |
| FINAL | `7_final_test.yaml` |

O original da geração permanece preservado. O ID para execução aparece na tela.
**Pasta para salvar os planos** permite escolher outro
destino, que também fica salvo; nesse caso, configure o executor para esse destino.

Ao terminar, confira o resumo e use **Criar rascunho com cenário e plano**. A ficha
recebe cópias dos arquivos exatos, sem precisar fazer upload novamente.

### Importar arquivos existentes

1. Em **Novo teste**, importe `*_equipe.yaml` do Commander, confira a seed e
   associe opcionalmente o YAML do plano otimizado utilizado.
2. Crie o rascunho. A duração vem de `duration_min`; altere o limite se necessário.
   A FINAL do PDF tem 20 minutos; um arquivo de 30 minutos continua com 30 e
   recebe um aviso de divergência.
3. Informe a data/hora em São Paulo e o tempo consumido em **segundos**.
   Desmarque as chegadas, identificações, capturas e entregas que falharam.
   Para uma interrupção, selecione as tarefas restantes e marque o grupo como
   não realizado. Os resultados são independentes: não há desmarcação em cascata.
4. Adicione incidentes, suas áreas/objetos e causas. Para manipulação deduzida,
   use um identificador por processo, como `captura_objeto4_WS2_1`.
   Repetir esse identificador na mesma tentativa cobra apenas uma dedução;
   um novo processo usa outro identificador.
5. Marque **Usou seguidor de linha** somente se o robô seguiu linha. Sem seguir
   linha, o painel acrescenta 20% automaticamente. Cada incidente de fita aplica
   automaticamente a faixa correspondente, sem pedir pontos ou justificativa.
   **Perfect Run** é marcado automaticamente quando todas as tarefas estão
   realizadas dentro do tempo e não há nenhum incidente na tentativa avaliada.
   Qualquer incidente retira automaticamente esse bônus. Você também pode marcar
   ou desmarcar **Perfect Run** manualmente; essa escolha fica salva até clicar
   em **Voltar ao automático**. A escolha manual altera somente o bônus de
   Perfect Run e habilita o bônus de tempo restante. Sem Perfect Run, o tempo
   continua registrado, mas não acrescenta pontos. Os descontos dos incidentes
   continuam sendo aplicados.
6. Confira a memória de cálculo, marque a revisão e finalize. Qualquer edição
   posterior ao aceite da revisão exige revisar novamente. É possível finalizar
   com decisões pendentes; esses registros continuam provisórios e fora das
   estatísticas até serem corrigidos e revisados.

Os campos fora dos formulários salvam a cada alteração confirmada pelo navegador
(campos de texto ao perder o foco ou pressionar Enter). Incidentes e novos
ajustes salvam ao enviar seus formulários. Aguarde a indicação **Salvo
automaticamente** antes de fechar a aba. Para continuar depois de fechar, abra
o rascunho na tela **Histórico**.

## Pontuação e interpretações

A versão atual é `CBR-2026-local-v4`, baseada no PDF
`CBR_RoboCup_Work_Rulebook_2026.pdf`, seções 5.5 e 6. O painel é um registro de
treinos com as interpretações abaixo preservadas em cada teste:

| Parcela | Valor |
|---|---:|
| Chegada a cada área ativa distinta, sem start/finish nem repetição por retorno | 100 |
| Identificação por objeto | 50 |
| Captura por objeto | 100 |
| Entrega correta por objeto | 100 |
| Perfect Run: tarefas dentro do limite e nenhum incidente, ou seleção manual | 30 por objeto |
| Tempo restante, somente com Perfect Run ativo | 1 por segundo |
| Queda / intervenção humana | −100 por ocorrência |
| Manipulação incorreta / colisão leve | −50 por ocorrência |
| Manipulação deduzida | −50 por processo afetado |
| Navegação sem seguidor de linha | +20% |
| Fita: uma / duas / três ou mais colisões | −5% / −10% / −25% |

O cálculo é **base − tarefas não realizadas − penalidades fixas + perfeição +
tempo + ajustes excepcionais**, depois **+20% se não seguiu linha**, depois
**−5%, −10% ou −25% por fita** e, por fim, a redução de colisão grave quando aplicável.
Incidentes são descontados separadamente e não apagam resultados já alcançados.
Não há piso zero nem arredondamento implícito. Desclassificação zera o resultado.

Colisão grave encerra a tentativa: escolha encerrar com metade dos pontos ou
reiniciar. Na segunda tentativa, só é permitido encerrar. Um reinício manual
também está disponível: a tentativa anterior fica preservada, seus pontos e
penalidades são descartados e seu tempo continua consumido. Há no máximo um
reinício. O tempo da primeira tentativa pode ser corrigido no bloco de consulta;
os valores anteriores continuam preservados nas revisões do histórico.

O uso de seguidor de linha é uma opção por tentativa, desmarcada por padrão.
O bônus de 20% usa o subtotal com tarefas conquistadas, penalidades fixas,
perfeição, tempo e eventuais exceções. O desconto de fita usa esse subtotal com
o bônus de navegação. As faixas de fita não se somam: duas colisões descontam
10%, três ou mais descontam 25%. Assim, 3.300 pontos sem seguir linha tornam-se
3.960; uma colisão com fita desconta 198 e resulta em **3.762 pontos**.

Não é necessário criar ajuste de arbitragem para esses casos. O bloco recolhido
**Simplificações e exceções (opcional)** serve apenas para decisões sem valor
fixado no regulamento, como uma simplificação combinada com os árbitros.

Decisões da primeira tentativa podem ser resolvidas após o reinício, mas seus
valores não entram na pontuação da segunda tentativa. A regra de colisão grave
aplica 50% ao subtotal após navegação e fita e sem bônus de conclusão. Cada ficha permite
consultar todas essas regras e interpretações; ela não depende de mudanças futuras
nos arquivos originais nem nas constantes do programa.

Na versão v3, incidentes impedem o Perfect Run automático e a marcação manual
tem prioridade até voltar ao automático. A escolha é preservada nas revisões e
exportações. Fichas antigas adotam essa regra ao serem abertas para edição;
resultados e revisões anteriores mantêm o cálculo original.

O exemplo `tests/fixtures/FINAL_seed8_equipe.yaml` tem dez objetos e oito áreas:
**3.300 pontos de tarefas + 300 de perfeição**, antes do bônus de tempo e ajustes.

As fichas `v1` conservam o cálculo original, inclusive os ajustes manuais já
registrados. Ao abrir um rascunho antigo ou criar uma correção, use **Atualizar
ficha para cálculo automático** para gerar uma revisão com as regras novas.
Essa atualização substitui os ajustes manuais de fita e navegação pelo cálculo
automático e começa com a opção de seguidor de linha desmarcada; confira-a antes
de finalizar. As revisões anteriores continuam no histórico e nas exportações.

## Histórico, comparações e recuperação

O SQLite fica por padrão em `data/tests.sqlite3`, ignorado pelo Git.
`ROBOT_TEST_DB=/caminho/tests.sqlite3` permite escolher outro local.
Revisões são acrescentadas, nunca sobrescritas. Alterações simultâneas da mesma
ficha são recusadas: reabra o registro pelo Histórico para carregar a versão atual.

No Histórico, filtre missão, seed, situação e período. Consulte qualquer revisão,
baixe os YAMLs originais, retome rascunhos ou crie uma correção com justificativa.
Uma correção abre uma nova revisão como rascunho, retirando temporariamente o
registro das estatísticas até nova finalização. A anulação exige justificativa e
preserva todas as revisões anteriores; também é possível corrigir uma anulação.

As estatísticas usam apenas a revisão atual de registros finalizados e sem
pendências. Em **Resultados por missão**, escolha uma missão: indicadores,
evolução da pontuação, tempos e perdas pertencem somente a ela. O resumo padrão
reúne todas as seeds dessa missão. O filtro **Cenário comparável** permite limitar
os resultados à mesma seed, conteúdo semântico do cenário, limite e regras
completas. Comentários e formatação do YAML não alteram a identidade; mudanças
no conteúdo alteram. O desvio padrão exibido é populacional, e os tempos médios
consideram somente missões marcadas como concluídas.

A aba **Erros acumulados · todas as missões** reúne os erros de todas as missões
e seeds, independentemente do filtro da outra aba. Exibe ocorrências e testes
afetados por tipo de incidente, incluindo tentativas anteriores a um reinício.
Uma tabela separada soma os pontos perdidos por tarefas não realizadas,
penalidades e ajustes, considerando somente as tentativas que entram na pontuação.

O banco recebe um backup diário em `data/backups/`, usando a data de São Paulo
na primeira abertura do dia. Os backups locais ficam no mesmo disco: use a
exportação para guardar uma cópia em outro dispositivo.

Em **Histórico → Exportar ou restaurar histórico**:

- **JSON** preserva arquivos importados, regras, cálculo e todas as revisões.
  Para recuperar, importe esse arquivo pela mesma tela. A restauração é
  transacional, faz backup antes e combina os dados sem apagar os existentes.
  Revisões idênticas são ignoradas; conflitos cancelam toda a restauração.
- **CSV** exporta tabelas de revisões, tarefas e incidentes/ajustes para análise.
  Use JSON para restaurar o painel. Textos com prefixo de fórmula são escapados
  nos CSVs para abertura em planilhas.

Para recuperar diretamente um backup SQLite, pare o painel, preserve o banco atual
e copie o backup desejado para `data/tests.sqlite3` antes de reiniciar.

## Validação

```bash
cd ~/ros2_ws
.venv/bin/python -m pytest src/cbr_work/tools/robot_test_dashboard/tests -q
```

Os testes cobrem importação, independência dos resultados, deduções por processo,
limites de tempo, reinício, colisões graves, pendências, revisões, concorrência,
backups, restauração transacional, separação estatística e fluxos da interface
via [Streamlit AppTest](https://docs.streamlit.io/develop/api-reference/app-testing).
