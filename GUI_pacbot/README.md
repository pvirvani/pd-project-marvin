# GUI_pacbot
> Graphical User Inteferface for the pacbot project

---

The next image shows the structure of the interface.

![Interface](https://i.imgur.com/NnXRGOH.png)

So, there are 3 main panels :
1. The **plan**, to show to the user the next actions to do for the robot and him.
2. The **scene**, the main execution, there is the 3D scene with the LEGOS, we can also navigate through the 3D scene with the arrows.
3. The **options**, to have a better interaction with the robot.

This resulted in this interface.

![Application](https://i.imgur.com/x9JDZ0N.png)

---

# 1. How to run the interface ?

---

## 1.1 Download NPM

First of all, the application run on vue js, so you need to install Vue.js but before, you need to install npm and nodejs.

> Note: Windows doesn't support [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite), hence you might see some errors and warnings!

### Linux
1. Add NodeJS Repo:
```
sudo apt update
sudo apt install curl dirmngr apt-transport-https lsb-release ca-certificates vim
curl -sL https://deb.nodesource.com/setup_18.x | sudo -E bash -
```

2. Install Latest Node.js on Ubuntu / Debian
```
sudo apt install gcc g++ make
sudo apt install nodejs
```

3. Check if everything was installed corrcetly:
```
node --version

npm --version
```

### Windows
https://phoenixnap.com/kb/install-node-js-npm-on-windows

---

## 1.2 Download all dependency modules

npm bring a tool to install all dependencies for a project at one time, you can just run the following command.

`npm install`

## 1.3 Run the development mode

To run the development mode, you need to run the following command

`npm run serve`

Then, you will start a development server, you just have to ctrl+clic on the link that appears to see the interface.

## 1.4 Build the application

In order to build the application, you need to be in the /gui directory and run the following command.

`npm run build`

It will update the dist directory, it contains the build application.

Then open your browser and write this in the url bar.

`http://localhost:8000`

---

# GUI Robot Integration

> Please Refer to this [Guide](Robot_Integration_Guide.md)

&nbsp;
