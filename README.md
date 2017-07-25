# Aplicação do ROS no Controle de Movimento de Robôs Equipados com Motores Dynamixel em Linux de Tempo Real 

__Categoria do Trabalho:__ Projeto de Fim de Curso

__Instituição:__ Universidade Federal de Uberlândia

__Curso:__ Engenharia Mecatrônica

__Aluno:__ Mateus Amarante Araújo

## Resumo

A EDROM é um grupo de pesquisa da Universidade Federal de Uberlandia que participa de diversas competições de robótica ao redor do mundo, já tendo ganhado diversos títulos. Apesar do sucesso, a equipe enfrenta dificuldades em realizar o controle de movimento dos seus robôs humanoides. Uma razão para isso é o fato das ferramentas computacionais utilizadas carecerem de recursos para implementar estratégias alternativas de controle. Para superar as limitações encontradas, este trabalho propõe uma solução genérica baseada no ROS, especialmente sua ferramenta chamada *ros_control*, para controlar o movimento não somente dos robôs da EDROM, mas de qualquer robô equipado com motores *Dynamixel*. Mostra-se que o novo sistema se destaca por ser modular, flexível e oferecer ferramentas que podem ser facilmente integradas com projetos de terceiros e utilizado em sistemas operacionais de tempo real de forma segura. O sistema é testado com sucesso em robô humanoide de 8GDL, sendo controlado por vários controladores genéricos e integrado com outros pacotes do ROS, como o rviz e a tf, usados para estimar o estado do robô e visualizá-lo em ambiente tridimensional. O novo sistema também é testado em Linux de tempo real, apresentando desempenho de controle de posição significamente melhor quando comparado ao sistema antigo em Linux de propósito geral.

## Conteúdo

Este repositório apresenta 4 ROS _packages_ principais:

* **dxl_robot_hw:** implementação de *DxlRobotHW*, *RobotHW* para comunicar com robôs equipados com motores *Dynamixel* + programa de teste com estrutura de uma junta, o *dxl_robot_hw_test*;

* **control_loop:** implementação do *ControlLoop*, classe que gerencia um *loop* de controle potencialmente de tempo real. O *RosControlLoop* comporta um *RobotHW* e um *ControllerManager*, executando, a cada iteração, a sequência *RobotHW::read()->ControllerManager::update()->RobotHW::write();*

* **bioloid_example:** teste com robô de 8GDL (3 em cada braço + 2 na cabeça) integrado com rviz e tf;

* **posvel_controllers:** Implementação de controladores que utilizam a *hardware_interface::PosVelJointInterface*. O *posvel_interface::JointTrajectoryController* e o *PosVelJointGroupController*.

O repositório também contém outros 2 pacotes ROS secundários:

* **control_timer_tests:** testes básicos com *control_loop::ControlLoop* (OBSOLETO)

* **ilc_feedforward_controller:** código preliminar para implementar o controlador desenvolvido por SCHWARZ e BEHNKE (2013) - <https://www.ais.uni-bonn.de/papers/RC13_Schwarz.pdf>

Enfim, também há 2 projetos CMake independentes do ROS:

* **dxl_tests:** testes isolados, como de registro de tempo de resposta, e de programas auxiliares, como de identificação estimação de frequência máxima de comunicação dos motores *Dynamixel*;

* **dxl_interface:** biblioteca auxiliar que envolve a Dynamixel SDK, abstraindo certos detalhes e adicionando novas funcionalidades. Por enquanto, somente a classe *dxl_interface::ModelSpec* está funcional, sendo utilizada no *DxlRobotHW*.

Em breve, o texto final será disponibilizado.

## Dependências

Além dos pacotes "padrão" do ROS, como a tf e o rviz, estes projeto depende dos seguintes *ROS packages*: `ros_control ros_controllers realtime_tools imu_filter_madgwick phidget_drivers ros-bioloid`

Todos podem ser encontrados no repositório ROS, exceto o `ros-bioloid`, encontrado em <https://github.com/dxydas/ros-bioloid> (ressalta-se que este projeto só faz uso dos desenhos CAD)

Também é necessário instalar a *Dynamixel SDK* (<https://github.com/ROBOTIS-GIT/DynamixelSDK>) e a *dxl_interface* desenvolvida (o README.md da pasta do projeto contém instruções de instalação)


