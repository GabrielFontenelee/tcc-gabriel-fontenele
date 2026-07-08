# TCC Robot Arm Simulation

Este diretorio contem os arquivos MATLAB/Simulink usados para simular o
controle do robo manipulador do TCC.

## Arquivos principais

- `TCC_run.m`: executa uma simulacao unica e gera figuras basicas.
- `main_automacao.m`: executa os cenarios de automacao e salva os arquivos
  `.mat` em `Graficos/`.
- `sim_TCC_2024.slx`: modelo Simulink principal.
- `trajetoria.m`: gerador de trajetoria polinomial usado pelos scripts.
- `traj.m`: script chamado pelo callback `InitFcn` do modelo Simulink.

## Como rodar

Abra o MATLAB neste diretorio ou execute:

```matlab
main_run
```

Para gerar todos os cenarios de comparacao:

```matlab
main_automacao
```

Depois de gerar os arquivos em `Graficos/`, use:

```matlab
plot_resultados_2
```

## Arquivos gerados

O Simulink pode criar `slprj/` e `*.slxc`, e os scripts podem criar a pasta
`Graficos/`. Esses itens sao gerados localmente e estao no `.gitignore`.
