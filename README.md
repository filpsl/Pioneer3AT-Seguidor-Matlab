# Pioneer 3-AT — Seguidor de Parede com LIDAR e MATLAB

Controle de um robô **Pioneer 3-AT** usando um **LIDAR RPLIDAR C1** e controladores
**PID** em MATLAB. O LIDAR é lido por um programa em C++ que reenvia as medições
por **UDP**; o MATLAB recebe esses dados, calcula a ação de controle e comanda as
rodas do robô pela biblioteca **Aria**.

Projeto extracurricular — Universidade de Brasília (UnB), Faculdade de Tecnologia / LEIA.

---

## Visão geral

O sistema tem três estágios. O LIDAR é capturado por `pioneer_lidar.cpp` (uma
versão modificada do *grabber* do SDK SLAMTEC), que reenvia cada amostra por UDP
para o MATLAB, onde os scripts de controle rodam.

```
RPLIDAR C1
   │ USB
   ▼
PC embarcado do robô ──[socat UDP:8089]──┐
                                         │ Wi-Fi
                                         ▼
PC remoto:  pioneer_lidar.cpp ──[UDP 127.0.0.1:5000]──▶ MATLAB
            (grabber do SDK)                            get_lidar.m
                                                        corredor.m / seguidor.m
                                                            │
                                            comandos Aria ──┘
                                                  │ TCP (socat:8101)
                                                  ▼
                                            Pioneer 3-AT (rodas)
```

Fluxo de dados: **LIDAR → `pioneer_lidar.cpp` → UDP `127.0.0.1:5000` → MATLAB**.

> O `pioneer_lidar.cpp` envia para `127.0.0.1`, portanto **ele e o MATLAB rodam na
> mesma máquina** (o PC remoto). Esse PC se conecta ao robô pela rede Wi-Fi.

---

## Estrutura do repositório

```
.
├── pioneer_lidar.cpp           # Grabber do SDK modificado: LIDAR → UDP 5000
├── corredor/
│   ├── corredor.m              # Experimento: centralização em corredor
│   └── get_lidar.m             # Parser dos datagramas UDP do LIDAR
├── seguidor-parede/
│   ├── seguidor.m              # Experimento: seguimento de parede
│   └── get_lidar.m             # Parser dos datagramas UDP do LIDAR
├── Manual_Robô_PIONEER_3AT.pdf # Manual de conexão
├── LICENSE
└── README.md
```

---

## Pré-requisitos

### Hardware
- Robô **Pioneer 3-AT** com computador embarcado.
- LIDAR **RPLIDAR C1** (SLAMTEC), conectado por USB ao computador embarcado.
- Rede **Wi-Fi** comum entre o PC remoto e o robô.

### Software (no PC remoto)
| Item | Observação |
|------|------------|
| Ubuntu / Zorin OS (Jammy) | Ambiente validado |
| GCC 11.4 | Para compilar o grabber e a Aria |
| MATLAB R2025b | Roda os scripts de controle |
| [SDK SLAMTEC](https://www.slamtec.com/en/support#rplidar-c1) | Driver e app do LIDAR |
| [Aria — fork `filpsl/Aria`](https://github.com/filpsl/Aria) | Fork ajustado para compiladores recentes |
| [`aria-matlab` — fork `filpsl/aria-matlab`](https://github.com/filpsl/aria-matlab) | Módulo de integração com o MATLAB |
| `socat` | Pontes serial→rede no computador embarcado |

A instalação da **Aria**, do **`aria-matlab`** e a configuração das pontes
`socat` estão detalhadas no `Manual_Robô_PIONEER_3AT.pdf`. Este README cobre a
parte específica do LIDAR e da execução dos experimentos.

---

## Compilação do grabber

O `pioneer_lidar.cpp` é o app *grabber* do SDK SLAMTEC modificado: além de ler o
LIDAR, ele abre um socket UDP e reenvia cada medição para o MATLAB. Ele depende
dos cabeçalhos do SDK (`sl_lidar.h`, `sl_lidar_driver.h`), então é compilado
**de dentro do SDK**.

```bash
# 1. Baixe e extraia o SDK do RPLIDAR C1 (site da SLAMTEC).
# 2. Substitua o main.cpp do app do grabber pelo arquivo deste repositório:
cp pioneer_lidar.cpp <sdk>/app/simple_grabber/main.cpp

# 3. Compile o SDK:
cd <sdk>
make

# 4. O binário fica em:
ls output/Linux/Release/      # -> simple_grabber
```

> Se o seu LIDAR não for o RPLIDAR C1, o SDK, o *baudrate* e o nome do app podem
> mudar — consulte a documentação do seu modelo.

---

## Configuração

Endereços e portas estão fixos no código. Antes de executar, ajuste conforme a
sua rede:

| O quê | Onde | Valor padrão |
|-------|------|--------------|
| IP do robô (Aria) | `corredor.m` / `seguidor.m` → `aria_init('-rh', ...)` | `192.168.0.3` |
| Porta UDP que o MATLAB escuta | `corredor.m` / `seguidor.m` → `porta_lidar` | `5000` |
| Destino UDP do grabber | `pioneer_lidar.cpp` → `sin_port` / `sin_addr` | `127.0.0.1:5000` |
| Porta `socat` do LIDAR | comando `socat` no robô e argumento do grabber | `8089` |

---

## Execução

Inicie os componentes **nesta ordem**:

**1. No computador embarcado do robô** — abra as duas pontes `socat`:

```bash
# Ponte para o robô (Aria) — ajuste a porta serial se necessário:
sudo socat TCP-LISTEN:8101,fork,reuseaddr /dev/ttyS0,raw,echo=0

# Ponte para o LIDAR (com o RPLIDAR C1 conectado na USB):
sudo socat UDP-LISTEN:8089,fork,reuseaddr /dev/ttyUSB0,raw,echo=0,b460800
```

**2. No PC remoto** — inicie o grabber apontando para o IP do robô. Ele passa a
enviar as medições para `127.0.0.1:5000`:

```bash
cd <sdk>/output/Linux/Release
./simple_grabber --channel --udp <ip_do_robo> 8089
```

**3. No MATLAB** — abra `corredor/corredor.m` ou `seguidor-parede/seguidor.m`,
confira o IP do robô em `aria_init` e execute o script. Ele pede o conjunto de
ganhos PID no console e roda o experimento.

Para encerrar: pressione `Ctrl+C` no terminal do grabber; o script MATLAB para
sozinho ao fim do tempo de simulação e zera as velocidades do robô.

---

## Protocolo UDP

O `pioneer_lidar.cpp` envia **um datagrama por ponto medido** para
`127.0.0.1:5000`. Cada datagrama é uma string ASCII com três campos separados
por vírgula:

```
angle, dist, quality
```

| Campo | Formato | Unidade | Descrição |
|-------|---------|---------|-----------|
| `angle` | `%.2f` | graus (0–360) | Ângulo da medição |
| `dist` | `%.2f` | milímetros | Distância ao obstáculo |
| `quality` | `%i` | — | Intensidade/qualidade do retorno |

Exemplo de datagrama: `123.45, 1820.50, 47`

Do lado do MATLAB, `get_lidar.m` lê todos os datagramas disponíveis na porta,
**descarta as leituras com `quality <= 0`**, agrupa os pontos por ângulo
(dentro de uma margem em graus) e retorna a **mediana** das distâncias de cada
ângulo pedido. Ângulos sem leitura válida retornam `-1`.

```matlab
% Exemplo: distâncias (mm) nos ângulos 270° e 315°, com margem de 1°
dists = get_lidar(udp_lidar, [270, 315], 1);
```

---

## Experimentos

Os dois scripts compartilham a mesma estrutura: leem o LIDAR via `get_lidar.m`,
aplicam um controlador **PID** com filtro derivativo e comandam as rodas com
`arrobot_setwheelvels`. Ao iniciar, cada um pede um conjunto de ganhos PID:

```
1 - PSO   2 - FLA   3 - EMP   4 - Manual
```

### `seguidor-parede/` — Seguimento de parede
Mantém o robô a uma distância de referência (`ref_dist = 60 cm`) de uma parede à
esquerda.

- **Sensores:** LIDAR a **270°** (parede lateral) e **315°** (diagonal frente-esquerda).
- **Controle:** erro = `ref_dist − distância(270°)`; saída do PID soma à velocidade
  da roda esquerda.
- **Emergência:** se a diagonal (315°) ficar abaixo de `dist_segura_cm = 40`, o
  robô gira para se afastar.

### `corredor/` — Centralização em corredor
Mantém o robô no centro de um corredor (distância à parede esquerda ≈ distância à
parede direita).

- **Sensores:** LIDAR a **270°/90°** (paredes lateral esquerda/direita) e
  **315°/45°** (diagonais frente-esquerda/frente-direita).
- **Controle:** erro = `distância(direita) − distância(esquerda)`; saída do PID
  ajusta as duas rodas em sentidos opostos.
- **Emergência:** se uma das diagonais ficar abaixo de `dist_segura_cm = 40`, o
  robô gira para o lado livre.

Ao final, ambos calculam métricas (IAE, SSE, saturação, rugosidade) e plotam as
distâncias e a ação de controle.

---

## Solução de problemas

| Sintoma | O que verificar |
|---------|-----------------|
| Grabber: `cannot connect to the specified ip addr` | A ponte `socat` do LIDAR (8089) está ativa no robô? O IP do robô está correto? |
| Grabber: `cannot bind to the specified serial port` | LIDAR conectado na USB? Porta correta (`/dev/ttyUSB0`) e *baudrate* `460800`? |
| `get_lidar` sempre retorna `-1` | O grabber está rodando e enviando? A porta `5000` está livre? Algum firewall bloqueando UDP local? |
| MATLAB: erro em `aria_init` / `arrobot_connect` | `addpath("/usr/local/Aria/matlab")` executado? Ponte `socat` do robô (8101) ativa? IP correto? |
| MATLAB: porta `5000` ocupada | Uma execução anterior deixou o `udpport` aberto — rode `clear` ou reinicie o MATLAB. |

---

## Limitações conhecidas / próximos passos

- IP do robô e portas estão **fixos no código** (ver seção *Configuração*); um
  arquivo de configuração único facilitaria a troca de rede.
- `get_lidar.m` está **duplicado** em `corredor/` e `seguidor-parede/`; poderia
  ser uma função compartilhada.
- O pipeline foi validado apenas com o **RPLIDAR C1**; outros modelos exigem SDK
  e *baudrate* diferentes.
- Não há **testes automatizados** — a validação depende do robô físico.

---

## Créditos

- **Autores:** Filipe Barbosa, Sérgio Cruz.
- **Código base dos controladores MATLAB:** Mario Andrés Pastrana Triana.
- **Orientação:** Prof. Daniel Muñoz — UnB / LEIA.

## Licença

Distribuído sob a licença **MIT**. Veja o arquivo [`LICENSE`](LICENSE).

## Referências

- [Manual do Pioneer 3-AT](https://www.inf.ufrgs.br/~prestes/Courses/Robotics/manual_pioneer.pdf)
- [SDK SLAMTEC — RPLIDAR C1](https://www.slamtec.com/en/support#rplidar-c1)
- `Manual_Robô_PIONEER_3AT.pdf` — manual de conexão deste projeto.
