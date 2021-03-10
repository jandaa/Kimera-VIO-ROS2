# Kimera-VIO-ROS2
A ROS2 implementation of [kimera-VIO-ROS](https://github.com/MIT-SPARK/Kimera-VIO-ROS) that runs within a docker container.

**NB:** This repository is *NOT* stable and in active develpment. 

**NB:** This repository is *NOT* yet functional.

## Kimera
This repository is based exclusively on the work by Rosinol et al including their ROS wrapper implementation which was used as a reference for development. For more information, please refer to their publication below.

- A. Rosinol, M. Abate, Y. Chang, L. Carlone, [**Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping**](https://arxiv.org/abs/1910.02490). IEEE Intl. Conf. on Robotics and Automation (ICRA), 2020. [arXiv:1910.02490](https://arxiv.org/abs/1910.02490).

## Docker

This library was build with docker in mind from the start. The dockerfile required to build the image is located under the .devcontainer directory but once it is more stable it will be moved to a seperate docker file.

## Development

The best way to develop is using the [devcontainer](https://code.visualstudio.com/docs/remote/containers) inside of visual studio code.

