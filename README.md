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

Coloque a nuvem de pontos em `data/points.laz` ou ajuste `LAZ_PATH` em
`config.py`. O arquivo LiDAR não está incluído no repositório. Sua origem,
licença e instruções de obtenção ainda precisam ser documentadas.

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
  config.py                    # Parâmetros e caminhos
  requirements.txt             # Versões das dependências diretas
  INSTALACAO_BIBLIOTECAS.txt    # Orientações detalhadas de instalação
  scripts/teste_lidar.py       # Script exploratório, separado da execução principal
  modules/                     # Leitura, processamento, A*, métricas e exportação
  data/                        # Dados locais, não versionados
  outputs/                     # images/, metrics/, trajectories/ e cameras/
```

A estrutura original com duas pastas de mesmo nome foi preservada nesta
primeira versão. A reorganização será tratada na próxima etapa.

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
