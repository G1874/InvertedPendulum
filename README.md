# InvertedPendulum

<div align = "center">
<video src = "https://user-images.githubusercontent.com/93823195/234073343-8e2aae81-15c9-49ab-a0d5-55e47947dc47.mp4">
</div>

### Description:
Code for control of an inverted pendulum on a cart.

<p align="center">
<img src="https://user-images.githubusercontent.com/93823195/231308479-37ab8817-c530-455a-9c54-906b93e2adaf.png" height="65%" width="65%">
</p>

### Stm32:
The pendulum is controlled by an stm32l152re microcontroller.

### GUI:
Graphical user interfece displays real time data read from stm32 via uart and displays it in graph form

<p align="center">
<img src="./Images/GUI.png" height="65%" width="65%">
</p>
  
### Control law:
The inverted pendulum is controlled using an LQR regulator.

<p align="center">
<img src="./Images/NaturalResponse.gif" height="65%" width="65%">
</p>
<p align="center">
<img src="./Images/ResponseGraph.png" height="65%" width="65%">
</p>
<p align="center">
<img src="./Images/ControlledResponse.gif" height="65%" width="65%">
</p>
  
### Acknowledgements:
- Project uses modified AS5600 magnetic encoder stm32 library made by [Nicholas Morrow](https://github.com/nicholasmorrow/AS5600)
