# Planejamento de caminhos 3D com LiDAR e A*

Autor: **Rhuan D. Santos**.

Código em desenvolvimento para um Trabalho de Conclusão de Curso (TCC).
O projeto processa uma nuvem de pontos LiDAR, constrói um mapa voxelizado
e planeja caminhos tridimensionais usando A*.

O objetivo geral do TCC inclui a avaliação complementar do seguimento dos
caminhos em uma simulação dinâmica de quadricóptero com controle baseado
em lógica fuzzy. Esta versão do repositório contém o planejamento e a
exportação de trajetórias; a simulação e o controlador não estão incluídos.

## Funcionalidades atuais

- Leitura de arquivos `.LAZ`, amostragem e normalização de coordenadas.
- Recorte opcional, voxelização e bloqueio do subsolo.
- Planejamento A* em três dimensões e suavização por spline.
- Métricas de comprimento, colisões por amostragem e distância a obstáculos.
- Visualização com Open3D e Matplotlib.
- Exportação de métricas em JSON e de trajetórias em CSV.
- Bancada 3D com navegação em órbita e POV, índice regional em disco e limites
  de pontos ajustáveis na janela.
- Seleção interativa de início/destino e configuração reproduzível de testes.
- Animação do caminho provisório do A*, com câmera fixa e cancelamento.
- Alternância entre nuvem original e superfície dos voxels ocupados da grade.
- Diagnóstico visual de colisões da spline, com navegação entre os voxels atingidos.

## Instalação e execução

Ambiente verificado: **Python 3.11.9 de 64 bits no Windows**.
No PowerShell, a partir da raiz deste repositório:

```powershell
Set-Location Projeto_LAZ_metricas/Projeto_LAZ_metricas
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m pip check
```

O cenário padrão usa `data/reboucas_alves_guimaraes_2017.laz`, formado pelas
quadrículas 3314-153 e 3314-154 do levantamento LiDAR de 2017 do GeoSampa.
Os arquivos LiDAR não estão incluídos no repositório. Obtenha-os pelo
[GeoSampa](https://novogeosampa.prefeitura.sp.gov.br/), na camada
**Imagens Aéreas e Sensoriamento → Articulação de Imagens → Modelo Digital de
Superfície — MDS 2017 — Articulação**, consultando cada quadrícula e usando
**Baixar arquivo do NAS via servidor**. Coloque os dois arquivos em `data/` e una-os:

```powershell
.\.venv\Scripts\python.exe scripts/unir_nuvens.py --saida data/reboucas_alves_guimaraes_2017.laz data/3314-153.laz data/3314-154.laz
```

A união preserva os registros de pontos e grava um JSON de procedência.
A referência dos dados está em `referencias_lidar.bib`; `CITATION.cff` trata da
citação do software. Para outra nuvem, ajuste `config.py`, inclusive a origem
de normalização e os pontos de teste, ou escolha o arquivo na bancada 3D.

Revise também os pontos inicial e final, o tamanho do voxel e a amostragem
em `config.py`, de acordo com os dados utilizados. Execute na mesma pasta:

```powershell
.\.venv\Scripts\python.exe main.py
```

As janelas de visualização são interativas; feche a janela atual para
permitir que o programa prossiga para a próxima etapa. As saídas são
gravadas na pasta `outputs/` ao lado de `main.py`, independentemente do
diretório de execução:

- `outputs/images/`: capturas das janelas do programa principal, usando **P**.
- `outputs/metrics/`: estatísticas em JSON.
- `outputs/trajectories/`: trajetórias em CSV.
- `outputs/cameras/legacy/`: configurações de câmera antigas preservadas.

As subpastas são criadas quando necessárias. Arquivos anteriores à organização
foram preservados em subpastas `legacy/`; as saídas não são versionadas.

## Estrutura

```text
Projeto_LAZ_metricas/Projeto_LAZ_metricas/
  main.py                      # Fluxo principal
  app.py                       # Bancada interativa local
  app_3d.py                    # Inspeção 3D, POV e carregamento regional
  config.py                    # Parâmetros e caminhos
  requirements.txt             # Versões das dependências diretas
  INSTALACAO_BIBLIOTECAS.txt    # Orientações detalhadas de instalação
  scripts/teste_lidar.py       # Script exploratório, separado da execução principal
  modules/                     # Leitura, processamento, A*, métricas e exportação
  tests/                       # Testes pequenos com nuvem sintética
  data/                        # Dados locais, não versionados
  outputs/                     # images/, metrics/, trajectories/ e cameras/
```

A estrutura original com duas pastas de mesmo nome foi preservada nesta
primeira versão. A reorganização será tratada na próxima etapa.

## Bancada 3D com inspeção regional (recomendada)

Na pasta que contém `main.py`:

```powershell
.\.venv\Scripts\python.exe app_3d.py
```

O visualizador usa Open3D e renderização gráfica por GPU. Isso acelera a
visualização; o algoritmo A* continua na CPU.

1. Use **Abrir / indexar nuvem**. Na primeira abertura, a nuvem inteira é lida
   em blocos para criar um índice espacial em `outputs/cloud_cache/`. Nas
   seguintes, o índice é reutilizado. O arquivo original não é modificado.
2. Use **Inspecionar região** e clique em uma superfície na visão geral. O
   programa carrega os pontos originais do raio selecionado e aproxima a câmera.
3. Alterne entre **Mapa inteiro**, **Vista superior da região**, **Órbita da
   região** e **Entrar em POV**. No POV, arraste para olhar, use WASD para mover
   e Q/E para descer/subir. Velocidade e raio de detalhes são ajustáveis.
4. Com os detalhes carregados, use **Selecionar início** ou **Selecionar destino**
   e clique no ponto desejado. Ctrl+clique repete o modo ativo. A posição é
   associada a um ponto original da região carregada, não à prévia global.
   As coordenadas relativas e originais são exibidas; ajuste a altura adicional.
5. Role o painel para acessar o planejamento. Confira amostragem e voxel, use
   **Preparar grade do A*** e depois **Planejar entre os pontos**.

A seção **Quantidade de pontos visualizados** permite ajustar a prévia global
e o limite de carga regional, de 1 mil até 50 milhões de pontos, e mostra a
quantidade efetivamente exibida. Os valores iniciais são 300 mil e 2 milhões,
respectivamente. Use **Aplicar limites de visualização** após editar os valores.
Alterar a prévia reabre o índice (criando outro cache quando necessário) e limpa
a seleção e a grade. Para uma nuvem de 5,2 milhões de pontos, uma prévia de
6 milhões permite exibir todos os pontos do mapa inteiro. Valores maiores
aumentam o consumo de memória e podem reduzir a fluidez da navegação.

As regiões de inspeção usam todos os pontos originais no raio solicitado.
O limite regional considera os blocos consultados, antes do recorte esférico,
e pode superar a quantidade efetivamente exibida. Quando esse limite é excedido,
a carga é recusada: aumente o limite ou reduza o raio; não ocorre amostragem
silenciosa dos detalhes. O índice conserva as coordenadas inteiras LAS e seus
fatores de escala. A GPU desenha coordenadas locais; a seleção projeta os pontos
da região na câmera e recupera uma coordenada original, sem interpolar no
buffer de profundidade. O cursor procura pontos em um raio de 5 pixels, com
preferência de profundidade em faixas subpixel. Aproxime a câmera para
distinguir objetos próximos antes da seleção.

O carregamento regional em POV acompanha a posição da câmera. Esta versão
usa blocos espaciais em disco, uma prévia global e uma região detalhada móvel;
não é uma implementação de octree com múltiplos níveis de detalhe. Não existe
garantia de FPS, detecção automática de cabos ou colisão da câmera com a cena.
O botão de captura PNG dessa bancada está temporariamente desativado: o
renderizador Vulkan instalado exibiu a cena corretamente na janela, mas
produziu imagens vazias na captura fora da tela. A exportação de métricas e
trajetórias e as funções de captura dos visualizadores anteriores são separadas.

### Alternar pontos e voxels

Na seção **Representação do mapa**, selecione **Pontos originais** ou
**Voxels ocupados do A***. Prepare a grade antes de usar a segunda opção.
A malha é construída no processo de trabalho na primeira seleção e reutilizada
nas alternâncias seguintes. Ela representa a ocupação da grade preparada,
incluindo o subsolo bloqueado, com cubos do tamanho usado naquela preparação.
Somente as faces externas são desenhadas; faces entre dois cubos ocupados são
omitidas. Isso não muda a ocupação nem cria uma nova amostragem do mapa.

A alternância mantém a câmera, os caminhos e as marcações de colisão. A seleção
de posições e a inspeção de pontos requerem a vista **Pontos originais**.
Trocar de arquivo, preparar outra grade ou liberar a grade remove a malha antiga
e restaura a vista original. Editar o tamanho do voxel exige preparar novamente
a grade; o painel informa a aresta usada na malha atualmente carregada.

### Inspeção das colisões da spline

Após planejar na bancada 3D, role o painel até **Colisões da spline | Diagnóstico**.
As caixas laranja representam os voxels ocupados atingidos; os pontos magenta
são as amostras da spline detectadas dentro desses voxels. O A* permanece
vermelho e a spline azul. A opção **Mostrar colisões detectadas** permite
ocultar e restaurar a sobreposição.

Use **Próxima colisão / aproximar** e **Colisão anterior** para percorrer os
voxels em ordem de índice da grade, com aproximação da câmera e solicitação
dos detalhes regionais. O painel informa o índice e as coordenadas locais e
originais do centro da caixa; esse centro não representa o ponto de contato.
O tamanho das caixas vem da grade utilizada no resultado, mesmo que o campo
de tamanho do voxel tenha sido editado depois do planejamento.

A validação atual continua sendo amostrada, sem alteração do spline ou do
passo de validação. As marcações mostram somente colisões detectadas por esse
método. Os campos `pontos_colisao_m` (coordenadas locais) e `segmentos_colisao`
(índices a partir de zero dos segmentos entre pontos da trajetória) são salvos
no JSON de métricas, junto com `voxels_colisao`. Ao alterar os pontos da
trajetória ou iniciar uma nova busca, as marcações anteriores são removidas.

### Acompanhamento da busca A*

Configuração inicial de testes: mapa `data/reboucas_alves_guimaraes_2017.laz`,
voxel de 3 m e amostragem de 1 em 10. Ao abrir esse mapa na bancada 3D, são
restaurados início `(170.440, 291.920, 29.590)` e destino
`(942.260, 550.330, 60.060)` em coordenadas locais, ambos já incluindo 5 m
acima do ponto escolhido. Use **Preparar grade do A*** para validar e depois
planeje. O botão **Restaurar pontos de teste** recupera essas posições após
edições. A restauração é restrita ao arquivo configurado em `TEST_MAP_PATH`;
as posições originais são convertidas usando a origem do índice carregado.

Ao clicar em **Planejar entre os pontos** na bancada 3D, o painel fica oculto
e a navegação manual é bloqueada até o resultado, erro ou cancelamento. O mapa
permanece como contexto: uma linha laranja mostra o caminho provisório desde
o início até o nó atual, reconstruído pelos predecessores do A*. A linha cresce,
encolhe ou troca de ramo quando a busca muda de alternativa; ela ainda não é
um caminho completo até o destino. Uma esfera amarela marca sua ponta. A câmera
enquadra início e destino uma vez e permanece fixa durante a busca, sem saltar
entre nós. O botão **Cancelar busca** permanece
disponível; cancelar encerra o processo e exige preparar novamente a grade.
Ao encerrar o acompanhamento, o painel e a vista anterior são restaurados,
e o resultado mostra os caminhos e o diagnóstico de colisões.

Atualizações do caminho provisório são enviadas aproximadamente a cada 150 ms.
A fila tem dois quadros e
descarta atualizações quando cheia, sem bloquear o A*. A animação não mostra
todos os nós explorados e não garante uma solução até o destino. Buscas muito rápidas
podem terminar antes de haver uma animação perceptível; não há atraso artificial.
O modo de acompanhamento continua bloqueado durante a avaliação da spline.

O campo `tempo_planejamento_s` inclui o custo de instrumentação da busca;
`tempo_envio_animacao_s` registra o tempo nas chamadas de envio, sem medir todo
o custo da renderização ou da comunicação assíncrona. Para comparar desempenho,
use a mesma configuração de animação. O JSON registra `animacao_busca`.

O índice consome espaço adicional em disco e pode ser recriado a partir do
arquivo original. Dados e caches continuam excluídos do Git.

**Precisão de visualização e resolução do planejamento são independentes.**
O padrão atual do planejador usa um ponto a cada dez e voxels de 5 m. Ele pode
omitir estruturas finas na amostragem, e seus pontos de caminho ficam nos
centros dos voxels. Ver um cabo na inspeção não prova que ele foi representado
adequadamente na grade. A resolução espacial, a preservação de obstáculos
finos e as margens do VANT precisam de avaliação específica antes de qualquer
conclusão sobre desvio de cabos. Casas decimais nas coordenadas representam
a quantização do arquivo, não uma certificação da exatidão do levantamento.

O visualizador e o planejador usam a mesma origem (mínimos do cabeçalho LAS)
nessa bancada. A execução tradicional preserva sua normalização anterior.

## Bancada anterior com gráficos (protótipo preservado)

Na pasta que contém `main.py`, execute:

```powershell
.\.venv\Scripts\python.exe app.py
```

1. Selecione o arquivo, a amostragem e o tamanho do voxel e clique em
   **Carregar mapa**. O recorte opcional usa os limites de `config.py`.
2. Com **Selecionar início** ativo, clique perto de um ponto na vista superior
   à esquerda. A seleção passa automaticamente para o destino.
3. Clique no destino. Defina a altura de cada ponto e use **Aplicar alturas
   e validar**. Coordenadas relativas, voxels e validade aparecem no painel.
4. Use **Planejar caminho** para executar o A* e comparar as trajetórias.
5. Use **Salvar imagem da bancada** para salvar os dois gráficos em
   `outputs/images/`.

A seleção usa a posição do ponto mais próximo da prévia em XY, dentro de
18 pixels do clique. A prévia contém até 40 mil pontos para manter a tela
responsiva; o processamento e a validação usam a nuvem amostrada completa
e a grade correspondente. A altura é relativa à superfície selecionada,
que não é necessariamente solo. As coordenadas são normalizadas; a origem
original e os parâmetros são registrados nas métricas de cada experimento.
O A* retorna centros de voxels: eles podem diferir do marcador selecionado
em até metade do voxel em cada eixo.

O carregamento e o planejamento rodam em um processo separado. O mapa fica
em memória para novas buscas; alterar seus parâmetros exige recarregá-lo.
**Cancelar operação** encerra o processo e exige recarregamento. Nesta versão,
o progresso indica etapas e o resultado final; ainda não há animação da
expansão, pausa, estimativa percentual ou aceleração por GPU.

Os gráficos mostram a prévia da nuvem e os caminhos, não todos os voxels
ocupados. A validade dos pontos é conferida na mesma grade usada pelo A*.
Se a spline apresentar colisão na validação por amostragem, as métricas e
a comparação visual são mantidas, mas o CSV de seguimento não é exportado.
Essa proteção também se aplica à execução tradicional via `main.py`.

Tkinter faz parte da instalação padrão do Python para Windows; Matplotlib
e SciPy já estão em `requirements.txt`. Não são necessárias bibliotecas novas.

Para executar os testes sem abrir janelas ou usar o arquivo LiDAR local:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

## Estado de validação

Em 30/10/2026 foram verificados os imports das dependências, a ausência de
conflitos reportados por `pip check` e a leitura de uma amostra de dez pontos
do arquivo LAZ local. O usuário também confirmou a execução do programa.
As dependências indiretas não estão congeladas neste `requirements.txt`.

A verificação de colisão usa amostragem espacial e não constitui garantia
formal de segurança de voo. A avaliação da metodologia e do seguimento dos
caminhos faz parte do desenvolvimento do TCC.

## Citação e condições de uso

Se utilizar este código ou resultados derivados em trabalhos acadêmicos,
cite o autor e este repositório. O arquivo `CITATION.cff` fornece os dados
para a citação do software. A referência bibliográfica do TCC será adicionada
quando o título definitivo e os dados da publicação estiverem disponíveis.

Referência inicial sugerida:

> SANTOS, Rhuan D. Planejamento de caminhos 3D com LiDAR e A*.
> Código-fonte em desenvolvimento. Disponível em:
> https://github.com/Trembonauta/lidar-astar-3d.

Este repositório ainda não possui uma licença de reutilização definida.
A solicitação de citação não substitui uma licença; entre em contato com o
autor para combinar permissões de uso e redistribuição.
