# Sphere-Meshes for Real-Time Hand Modeling and Tracking

- **Video**: http://lgg.epfl.ch/publications/2016/HModel/video.mp4
- **Paper PDF**: http://lgg.epfl.ch/publications/2016/HModel/paper.pdf
- **Dataset**: http://lgg.epfl.ch/publications/2016/HModel/teaser.zip

## Disclaimer
To obtain the results shown in the video proper hardware is necessary:
- **Windows 8**
- Intel Core **i7 @4GhZ**
- CUDA Graphic card (**NVIDIA GTX980** used in our demo)
- Inter RealSense depth camera 

Other notes:
- note the software must be compiled in **64bits**
- **Wristband** color calibration (make sure the wristband is detected robustly otherwise the tracking might not perform as effectively, you can check this by enabling "show wband" in the hmodel_atb application)

## BibTex
	@article{hmodel,
    title = {Sphere-Meshes for Real-Time Hand Modeling and Tracking}, 
    author = {Anastasia Tkach and Mark Pauly and Andrea Tagliasacchi}, 
    journal = {ACM Transactions on Graphics (Proceedings of SIGGRAPH Asia)}, 
    year = {2016}}
	
## Running "Teaser" Sequence

in `hmodel/apps/hmodel_atb/main.cpp`

- change `21 | std::string sequence_path` to the path to teaser dataset in you machine
- change `22 | std::string data_path` to the path to `hmodel/data/` folder on your machine
- set `17 | bool benchmark`to `true`

## Running Live

in `hmodel/apps/hmodel_atb/main.cpp`

- set `17 | bool benchmark` to false
- wear a blue wristband and make sure that the wristband is always in the view of the sensor
- c++ implementation of automatic hand model calibration is coming
- for now to approximately adjust the model for your hand
    * press `key 1` for *uniform scaling up* 
    * press `key 2` for *uniform scaling down* 
    * press `key 3` for *width scaling up* 
    * press `key 4` for *width scaling down* 
    * press `key 5` for *thickness scaling up* 
    * press `key 6` for *thickness scaling down* 

