# dqrobotics-matlab

The DQ Robotics library in [Matlab](https://www.mathworks.com/).

## Installation

Assuming that you already have Matlab installed on you computer, download the most recent Matlab toolbox of DQ Robotics [here](https://github.com/dqrobotics/matlab/releases/latest). 

After downloading the file dqrobotics-YY-MM.mltbx, where YY-MM stands for the year and month of release, just open it and Matlab should copy the files to the folder `Toolboxes/dqrobotics-YY-MM` in your `$HOME` folder and appropriately set the Matlab path. 

To test if the toolbox was installed correctly, just go to the prompt and type 

```ans
>> DQ
ans = 
        0     
```

If you receive an error instead, it means that the toolbox was not properly installed and should open an issue [here](https://github.com/dqrobotics/matlab/issues).

### Development branch

Those wanting the results of our latest developments can checkout the master branch of the [matlab repository](https://github.com/dqrobotics/matlab). In order to use DQ Robotics on your MATLAB installation, and supposing you did the checkout at **[PATH_TO_DQ_ROBOTICS_FOLDER]**, just add 

```
[PATH_TO_DQ_ROBOTICS_FOLDER]/matlab/
```

and subfolders to your MATLAB path.

Note however, that the development branch is unstable and should not be used in production environments.

## Updating your dqrobotics-matlab installation

DQ Robotics follows a biannual release schedule. Therefore, there will be new releases each April and October. Just go to the [download](https://github.com/dqrobotics/matlab/releases/latest) page and get the latest version. 

## Examples

There are several examples inside the folder `$HOME/Toolboxes/dqrobotics-YY-MM/examples`. They range from robot modeling to constrained robot control. 

## Getting help

All classes and methods are well documented. To access their documentation, just type `help CLASS_NAME`. For instance, `help DQ` will show all methods available in the class DQ with their corresponding description. To show more information about a particular method or property, it suffices to type `help CLASS_NAME/method`; for instance, `help DQ/plot`  will show the help for the plot  function of the DQ class.

Although the library is self-contained and it suffices to know only basic dual quaternion algebra and robot modeling and control techniques to use it, there is a great amount of theory behind the controllers and modeling techniques implemented in DQ Robotics. Therefore, several methods and classes point to relevant references in the literature and users that want to better understand the implementation may find those references very useful.
