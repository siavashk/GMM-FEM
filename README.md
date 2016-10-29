
# GMM-FEM Registration #

## WHAT DOES THE PACKAGE CONTAIN?

The code contains programs to perform rigid, affine and non-rigid registration
of 3D point clouds. The rigid and affine registration are performed using the 
algorithm presented by Myronenko and Song [1]; whereas the non-rigid 
registration is based on the work presented by Khallaghi et al. [2] and Fedorov 
et al. [3].

If this code contributes to a project that leads to a scientific publication, 
please acknowledge this fact by citing the relevant papers [2,3].

[1] Myronenko and Song., "Point set registration: Coherent point drift." 
Pattern Analysis and Machine Intelligence, IEEE Transactions on 32(12), 2010.

[2] Khallaghi et al., "Biomechanically Constrained Surface Registration: 
Application to MR-TRUS Fusion for Prostate Interventions", Medical Imaging, IEEE Transactions on 34(11), 2015.
 
[3] Fedorov et al., "Towards an open source framework for image registration
in support of MRI/ultrasound-guided prostate interventions", International Journal of Computer Assisted Radiology and Surgery, 2015.

## BUILD INSTRUCTIONS

### KNOWN DEPENDENCIES

Two libraries, Tetgen and Maslib, need to be built in order to run the
algorithms.  We have already supplied these packages in /GMM-FEM/ThirdParty.  

### BUILDING TETGEN AND MASLIB for All Platforms

Makefiles are provided in the root folders of the packages for building the
libraries and MATLAB mex files.  They can be invoked using the GNU make
build system:

  $ make -f Makefile.<platform>

where <platform> is either 'linux', 'mac', or 'windows'.  The utility will
attempt to guess where MATLAB is installed so it can find the required mex
libraries.  To specify a path, provide the root folder:

  $ make -f Makefile.<platform> MATLAB_ROOT=<path to matlab>

e.g. MATLAB_ROOT="C:/Program Files/MATLAB/R2013b"

#### Windows Users:

Note that the code relies on some C++11 features that are not yet fully
implemented in the VC++ compiler provided by Visual Studio 2013.  Therefore,
we suggest downloading MinGW64 for compiling:

   http://sourceforge.net/projects/mingw-w64/

When prompted, allow the installer to add the MinGW directory to your system
path.  This will allow you to use the "make.exe" program for compiling.  
Depending on the version of MinGW64 installed, you may need to invoke:

  $ mingw32-make -f Makefile.windows


### VERIFYING THE BUILD

Run /GMM-FEM/Scripts/fem_only_test.m to perform non-rigid FEM-based 
registration.


### TROUBLESHOOTING

If you get an error regarding not being able to find or link to the MATLAB
libraries, first verify that the correct MATLAB path is being detected.
Internal variables can be printed using

  $ make -f Makefile.<platform> vars

If the MATLAB path is incorrect, then pass in the appropriate MATLAB_ROOT
variable.  If the libraries still cannot be found, verify that you are
using a 64-bit compiler with 64-bit MATLAB.

If you get an issue similar to "Invalid MEX-file bvtree_build.mexa64 ... libstdc++.so.6: version `GLIBCXX_3.4.18' not found", it might be that Matlab is loading a different version of libstdc++. On Ubuntu machines, Matlab internally links libstdc++ to a version found under its own binary directory. A workaround is to change this linkage to point to gcc's std library, which on Ubuntu machines is usually found under /usr/lib/x86_64-linux-gnu.
