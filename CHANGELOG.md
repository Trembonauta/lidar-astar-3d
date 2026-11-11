# Histórico de alterações

## 2026-11-11 — Bancada interativa para planejamento LiDAR 3D

- Organização dos scripts exploratórios e das saídas geradas em pastas próprias.
- Separação do processamento em um pipeline reutilizável e processo persistente
  para preparar a grade, validar posições e planejar sem bloquear a janela.
- Bancadas interativas em Matplotlib/Tkinter e Open3D, com seleção de início e
  destino, altura adicional e validação das posições na grade.
- Navegação 3D em órbita, vista superior e POV, com índice espacial em disco,
  detalhes regionais e limites configuráveis da quantidade de pontos exibidos.
- Animação em laranja do caminho provisório reconstruído pelo A*, câmera fixa,
  bloqueio dos controles durante a busca e cancelamento disponível.
- Alternância entre a nuvem original e a superfície externa da ocupação voxelizada,
  incluindo o subsolo bloqueado, preservando câmera e sobreposições.
- Diagnóstico de colisões da spline: voxels em laranja, amostras em magenta,
  navegação entre colisões e registro dos detalhes no JSON de métricas.
- Bloqueio da exportação CSV de trajetórias suavizadas reprovadas pela validação.
- Script para unir quadrículas LAS/LAZ compatíveis sem subamostragem, com registro
  de procedência e conferência de contagem e limites.
- Cenário de testes Rebouças/Alves Guimarães, voxel de 3 m, posições restauráveis
  e referência BibTeX das quadrículas 3314-153 e 3314-154.
- Documentação de instalação, uso, dados e limitações; 34 testes automatizados
  aprovados com nuvens sintéticas, integração do worker e geometria de visualização.

A verificação de colisão continua amostrada, e o spline ainda não é corrigido
automaticamente. A captura PNG da bancada 3D está desativada por produzir imagens
vazias no renderizador disponível. Também foi observado erro de encerramento do
Vulkan em testes gráficos anteriores. A visualização não garante detecção de
cabos ou segurança de voo real; a simulação dinâmica e o controlador fuzzy ainda
não fazem parte desta versão. Nuvens LiDAR, ambientes virtuais e saídas geradas
permanecem fora do versionamento.
