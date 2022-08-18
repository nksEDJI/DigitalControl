%% Defining Process and Open loop response
num = 2;
den = [1 2 1];

process = tf(num, den);

%               2
%  H(s) = --------------
%         s^2 + 2 s + 1

% Open loop output:
figure(1);
step(process);

% Simulation time
time = 0:0.1:30;

%% Process Dynamics
addpath 'C:\TFS\Controladores Digitais\ProcessosIndustriais\src'


% Nesta seção obtemos os parâmetros que definem
% a dinâmica do processo:

% theta: atraso de transporte
% tau: constante de tempo
% k: ganho estático

dynamics = ProcessDynamics(process, time);
dynamics_parameters = dynamics.getDynamicsParameters();

%% Ziegler Nichols

zn = ZieglerNichols(dynamics_parameters);
zn_parameters = zn.getPIDParameters();

%% Controlador ZN

% Aqui utilizaremos o modelo do simulink disponi-
% bilizado para simulações: BaseControl.slx

% Dentro do simulink adicionei um bloco de con-
% trolador PID. Sendo que cada ganho recebe o 
% valor de uma variável do workspace:

% P : PROPORTIONAL_GAIN
% I : INTEGRAL_GAIN
% D : DERIVATIVE_GAIN

% Lembrando que I = Kp / Ti e D = Kp * Td
% Onde: 
% Kp: Ganho proporcional
% Ti: Tempo integrador
% Td: Tempo derivador

% Como estamos projetando um controlador PI, logo:
PROPORTIONAL_GAIN = zn_parameters.Kp;
INTEGRAL_GAIN = zn_parameters.Kp / zn_parameters.Ti;
DERIVATIVE_GAIN = 0;

%% Rodando a Simulação

% É importante ressaltar que os dados obtidos com
% a simulação serão salvos no workspace

sim('BaseControl');

%% Plotando as respostas no tempo

figure(2);
% A classe SimulationVisualizer possui métodos para plotar 
% os dados da simulação.
sgtitle('ZN Tunning')
%% Saída: y(t)

subplot(311);
SimulationVisualizer.plotOutput(Reference, OutputRead);

%% Sinal de Controle: u(t)

subplot(312);
SimulationVisualizer.plotControlSignal(Input);

%% Error: e(t)

subplot(313);
SimulationVisualizer.plotError(Error);

