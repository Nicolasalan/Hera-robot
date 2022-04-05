## Interagindo com as articulações de forma gráfica

O ROS disponibiliza uma interface gráfica para interagir com as articulações. Para isso, basta executar o seguinte comando:
Inicializar o launch file do controlador de articulações:
```
roslaunch hera_description visualize_model.launch
```
Em seguida, iniciar a interface gráfica no segundo terminal:
```
rosrun rqt_gui rqt_gui
```
Vamos ver como funciona: 
1. Selecionar `plugins` no menu principal -> Message Publisher
2. Selecionar o controlador desejado no `Topic` com o nome `/kinect_controller/command`, que é o tópico onde podemos enviar comandos para a junta, e clique no botão '+'.
3. Agora, ative o editor na seção abaixo clicando no botão de verificação que aparece à esquerda do nome do tópico e defina a taxa de publicação "publish rate" para 100.00.
4. Expanda o editor clicando no ícone '+' à esquerda da linha. Na coluna 'expression', experimente diferentes valores radianos e verifique como a junta se move na simulação.
5. Agora, tente colocar a próxima expressão na coluna 'expression': `sin(i/100)`. Confira o que acontece na simulação. Como você pode ver, você está fazendo a junta 1 do robô mudar seus valores usando uma onda senoidal. A letra i é a variável que rqt usa para tempo.
6. Em seguida, adicione uma exibição de `plotagem` acessando o menu 'Plugins', 'Visualization' e 'Plot'. Adicione o tópico /kinect_controller/command e clique no botão '+'.
7. Plote também o tópico `/kinect_controller/state/process_value`. Este tópico mostra a posição real da junta, portanto, você deseja que essas duas linhas sejam o mais iguais possível.
8. Abrir a `ferramenta de Reconfiguração` Dinâmica, indo em 'Plugins', 'Configuration' e 'Dynamic Reconfigure'. Esta ferramenta permite modificar dinamicamente alguns valores de configuração do robô. No painel esquerdo, clique no botão 'Expand All' e clique na opção `pid` para a junta. Ajuste os valores de pid para deixar as duas linhas o mais próximas possível.