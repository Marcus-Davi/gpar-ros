# GPAR ROS #

Pacotes ROS desenvolvidos pelo GPAR da UFC. 

Contato: davi2812@dee.ufc.br ou gpardee@dee.ufc.br

Pacotes **ROS** do repositório:

 * gpar_camera
    - Pacote de procesasmento simples de dados de câmera USB.
    
 * gpar_kalman
    - Pacote de implementação C++ de filtros de Kalman Linear e Estendido.
   
 * gpar_lidar
    - Pacote de construção de nuvem de pontos. Usa a **sick_ldmrs_laser**. Processamento, junção e aquisição de nuvem de pontos.

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
    
* gpar_misc
    - Pacote de utilitários.
    
    OBS: Todos estes pacotes requerem as bibliotecas ros-full, compiladas via codigo fonte ou instaladas diretamente.
    
    Em DESENVOLVIMENTO.



