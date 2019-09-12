# GPAR-OSDK ROS #

Repositorio contendo as ultimas atualizações de código da integração ROS-OSDK pro projeto EDP.

Os arquivos aqui contidos são pacotes ROS fornecidos pela DJI ou pela comunidade ROS. Pacotes com nome "gpar-*" foram desenvolvidos pelo laboratório.


Contato: davi2812@dee.ufc.br

Pacotes **ROS** do repositório:

 * gpar_dji
    - Pacote Principal. Contém implementação de missões; supervisor de posição; conversão de IMU para transformadas; Comunicação com *MSDK*. Usa **Onboard-SDK-ROS-3.8**
    
 * gpar_lidar
    - Pacote de construção de nuvem de pontos. Usa a **sick_ldmrs_laser**.

* gpar_k64f
  - Pacote não usado no projeto, mas desenvolvido para testes. Interface com K64F via Serial/Bluetooth. Usa **gpar_mcuserial**

* gpar_mcuserial
  - API Core da comunicação com K64F. Converte mensagens cruas recebidas via serial em mensagens ROS.

* Onboard-SDK-ROS-3.8
   - API de comunicação com Drones MATRICE ou WIND. Gera tópicos e serviços ROS para levantamento de dados via LIDAR e habilita operação sobre o drone.

* sick_ldmrs_laser 
   - API para comunicação com LiDAR SICK LD-MRS. Gera tópicos com as nuvens de pontos. Usa **libsick_ldmrs**.

* libsick_ldrms
    - Biblioteca Core baixo nível para comunicação com LIDAR SICK LD-MRS.
    
    OBS: Todos estes pacotes requerem as bibliotecas ros-full, compiladas via codigo fonte ou instaladas diretamente.
    
    Em DESENVOLVIMENTO.



