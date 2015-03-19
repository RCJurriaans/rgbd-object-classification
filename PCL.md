# Install #
  * download the all in one installer from [here](http://pointclouds.org/downloads/windows.html)
  * add the path to the variables as PCL\_ROOT
  * In Visual Studio add the following
    * c++ Include Directories:
      * $(PCL\_ROOT)\include\pcl-1.4;$(PCL\_ROOT)\3rdParty\Eigen\include;$(PCL\_ROOT)\3rdParty\Boost\include;%(AdditionalIncludeDirectories)
      * Eventueel extra 3rd party includes
    * Linker General Additional Lib Directories
      * $(PCL\_ROOT)\lib;$(PCL\_ROOT)\3rdParty\Eigen\bin;$(PCL\_ROOT)\3rdParty\Boost\lib;%(AdditionalLibraryDirectories)
      * Eventueel extra 3rd party libs
    * Linker Input Additional Dependencies
      * pcl\_io.lib;pcl\_common.lib;pcl\_sample\_consensus.lib;pcl\_segmentation.lib;%(AdditionalDependencies)
      * Eventueel extra libs zoals bijv. search
