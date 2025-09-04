
# Delivery_bridge

:rocket This is a ROS2 package. It is a bridge between the web interface and the ROS 2 environment.

This package have a node that start a HTTP server that provides a REST API, also start a Socket.IO server to provide a real-time communication between the web interface and the ROS 2 environment.


Tech Stack:

- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- [Python SocketIO 5.7.2](https://python-socketio.readthedocs.io/en/latest/)
- [FastAPI 0.105.0](https://fastapi.tiangolo.com/)
- [Sphinx 7.2.6](https://www.sphinx-doc.org/en/master/)
- [pre-commit 3.7.1](https://pre-commit.com/)


## Installation

To install the package, you need to have ROS2 installed in your machine. You can follow the instructions in the [ROS2 official website](https://docs.ros.org/en/humble/Installation.html).

After you have ROS2 installed, you can clone this repository in your workspace and build the package.

```bash
cd ~/colcon_ws/src
git clone https://github.com/cmauricioae8/delivery_bridge.git
cd delivery_bridge
pip install -r requirements.txt
cd ~/colcon_ws/
colcon build --packages-select delivery_bridge --symlink-install --allow-overriding delivery_bridge
source install/setup.bash
```

After of above configurations, test the project:

```bash
ros2 run delivery_bridge server_node
```

Go to local url http://127.0.0.1:9009 and check that all is going well


# For code quality

For code quality, we use pre-commit. To install it, run the following command:

```sh
pip install pre-commit
```

For new projects or every time you change the `.pre-commit-config.yaml` file, you need to run the following command:

```sh
pre-commit install # This will install the pre-commit hooks in your .git/hooks folder
```

Every time you will commit something, the pre-commit hooks will run. If you want to run them manually, you can run the following command:

```sh
pre-commit run --all-files
```


## Generate documentation

To generate the documentation with sphinx, run the following command:

```sh
cd docs
make html
```

If you want to generate a pdf, you first need to install latexmk

```sh
sudo apt-get -y install latexmk texlive-latex-recommended texlive-latex-extra \
    texlive-fonts-recommended texlive-fonts-extra
```

Then run the following command:

```sh
make latexpdf
```



pip install setuptools==58.2.0
