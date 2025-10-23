# Integration

This repo is intended to be built into your code as a submodule within your project repository. 
Barring that, you can just clone the entirey of the `Roobot-Over-IP` repo in your ros workspace. 

## Submodule Initalization
To setup ROI as a submodule to your project, enter your project repo and make it the current working directory of your command pormpt. Then run the git commands:

1. `mkdir external` (Or whatever you wish to store submodules in)
2. `git submodule add https://github.com/UA-NASA-Robotics/Roobot-Over-IP external/Roobot-Over-IP ./external/` (replace external here if changed in step 1)
3. `git submodule update --init --recursive`
4. Commit changes adding a `.gitmodules` file

Note git submodules work by initalizing a repo within your repo tied directly to 1 commit. With ROI, this will be the latest commit on main. Also note since ROI is a full repo, you can make changes and (depending on permissions) commit them back to ROI. 
We recomend a fork + pull request if you are outside of our organization.

You can run `git submodule update --remote` to fetch the latest changes from ROI into your sub-module.

### Platformio
You may also want to install platformio to update module firmware:
[PlatformIO](https://platformio.org/) extension for VSCode.

See [Developing for ROI](docs/ProgrammingROI.md) for more details.
