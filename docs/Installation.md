[<= Back to homepage](../README.md) 

**Table of content**
- [How to use this repository](#how-to-use-this-repository)
- [Creation of the training playground](#creation-of-the-training-playground)
  - [Install Docker Desktop on Windows](#install-docker-desktop-on-windows)
  - [Install Docker Engine on Linux](#install-docker-engine-on-linux)
- [Running the training playground.](#running-the-training-playground)

# How to use this repository
Copy this repository on you computer, either:
- downloading the given `.zip` file and extracting it on your computer,
- or by cloning the repository, if you have been given access to it.
Once your computer is setup, you can start with the exercise material chapters.

> :warning: If you have issues with the installation steps, feel free to contact: `e.bernardi@tudelft.nl`

# Creation of the training playground
The minimum required software to follow along the training is:
- VS Code editor, which you can install from here: https://code.visualstudio.com
- Docker, which you can install following instructions below.

Docker is a tool used to build, deploy, test, and otherwise work with software in an isolated environment, called a container. This diffs from a VM in that it shares the same linux kernal as your host operating system, making it faster to spin up and share host resources. By building or deploying software in this isolated environment, you can ensure many users, robots, or servers are running the same software with the same software versions across many instances. It gives you a controlled environment to work in that is reproducable on other developer’s machines and even work in a different (linux-based) operating system than your computer currently runs [[cit.](https://docs.nav2.org/tutorials/docs/docker_dev.html#preliminaries)]

## Install Docker Desktop on Windows
From this [page online](https://docs.docker.com/desktop/install/windows-install/) follow instructions on how to install Docker Desktop for Windows.

## Install Docker Engine on Linux 
From this [page online](https://docs.docker.com/engine/install/ubuntu/) you can get a minimal installation of Docker (Engine) on Ubuntu.


# Running the training playground.
Once all installations are complete, download this git repository as zip file, or simply clone to your preferred location on your machine.
Open the folder with VSCode and, when promped, accept "Reopen in container" prompt.
If you missed the prompt, you can always hit `F1` inside VSCode and start typing `"reopen container"` and then hit enter.
The editor will take some time to download and install all required steps to run the docker container.
Once completed, a new window of VSCode will automatically open and you can start the actual exercises!