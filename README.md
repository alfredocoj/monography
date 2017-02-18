PROJETO DO FILTRO DE KALMAN EM UM MANIPULADOR ROBÓTICO USANDO UM ALGORÍTMO GENÉTICO
======

by
André Cavalcante,
Ohnishi Laboratory,
Graduate School of Information Science,
Nagoya University,
Japan.

PDF: http://ir.nul.nagoya-u.ac.jp/jspui/bitstream/2237/21837/1/主論文.pdf

# Resumo
======
Este trabalho tem como objetivo utilizar o projeto do Filtro de Kalman para estimar as variáveis de estado de um manipulador robótico, uma vez que as mesma não estão disponíveis na prática. O problema do Filtro de Kalman é resolvido através da metodologia de controle robusto LQG, isso significa que tanto a incerteza com relação às variáveis de estado, quanto as pertubações são consideradas durante o projeto do controlador. Além disso, significa que o controlador projetado pode tolerar o erro de modelagem, sem degradar, significativamente, o desempenho do sistema de controle. Com relação à avaliação desta metodologia, analisou-se a resposta no domínio do tempo para uma entrada ao impulso unitário, avaliando tempo de pico, tempo de assentamento e erro de estado estacionário, e no domínio da frequência, avaliando parâmetros, tais como: Margem de Ganho, Margem de Fase, largura de banda, frequência de ressonância e pico de ressonância. De modo geral, o Filtro de Kalman projetado cumpriu seu papel de estimar os estados de forma satisfatória, e o desempenho e estabilidade do sistema compensado apresentaram comportamento desejado, mesmo com a presença de ruídos na planta e na medição, e de polos adicionais não modelados.

# Abstract 
======

This research aims to use design Kalman Filter to estimate the state variables of a robotic manipulator, since the same are not available in practice. The problem of the Kalman filter is solved using the methodology of robust control LQG, this means that both the uncertainty with respect to the state variables, as the disturbances are considered during the controller design. Furthermore, it means that the designed controller can tolerate the error modeling without significantly degrading the performance of the control system. Regarding the evaluation of this methodology, it has been analyzed the response in the time domain for a unit impulse input, evaluating peak time, settling time, and steady-state error, and in the frequency domain, evaluating parameters such as: Gain Margin, Margin Phase, bandwidth, resonance frequency and resonance peak. It has been observed that the Kalman Filter has fulfilled its role of estimating states, satisfactorily, and performance and stability of the compensated system had desired behavior, even in the presence of noise in the plant and noise in the measurement, and additional poles not modeled. 

# Conteúdo desse repositório
======

Este repositório conterá a monografia e todo o código relacionado a esta pesquisa.
