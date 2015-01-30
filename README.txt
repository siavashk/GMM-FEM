################################
# GMM-FEM Registration PACKAGE #
################################

###########################################################################################

--------------------------------
1 WHAT DOES THE PACKAGE CONTAIN?
--------------------------------

The code contains programs to perform rigid, affine and non-rigid registration
of 3D point clouds. The rigid and affine registration are performed using the 
algorithm presented by Myronenko and Song [1]; whereas the non-rigid 
registration is based on the work presented Khallaghi et al. [2] and Fedorov 
et al. [3].

If this code contributes to a project that leads to a scientific publication, 
please acknowledge this fact by citing the relevant papers:

[1] Myronenko and Song., "Point set registration: Coherent point drift." 
Pattern Analysis and Machine Intelligence, IEEE Transactions on 32.12 (2010):
2262-2275.

[2] Khallaghi et al., "Biomechanically Constrained Surface Registration: 
Application to MR-TRUS Fusion for Prostate Interventions", Submitted to 
Medical Imaging, IEEE Transactions on, June 2014.
 
[3] Fedorov et al., "Towards an open source framework for image registration
in support of MRI/ultrasound-guided prostate interventions", Accepted for 
Publication in IJCARS, 2015.

###########################################################################################

----------------------------
2 WINDOWS BUILD INSTRUCTIONS
----------------------------

----------------------
2.1 KNOWN DEPENDENCIES
----------------------

Two libraries, Tetgen and Maslib, need to be built prior to mex generation. 
We have already supplied these packages in /GMM-FEM/ThirdParty, however, 
the Windows user needs a make.exe and a Linux style find.exe to generate 
these libraries.
 
We recommend MinGW for both make and find executabels. This project can be
downloaded from: http://www.mingw.org/

Once MinGW is downloaded, we recommend changing the default mingw32-make.exe
to make.exe and adding the location of both make.exe and find.exe to environmental
variables.

------------------------------
2.2 BUILDING TETGEN AND MASLIB
------------------------------

Open the command prompt (cmd.exe) and navigate to /GMM-FEM/ThirdParty/maslib/
and /GMM-FEM/ThirdParty/tetgen1.4.3. Call make to generate these libraries.
If the Matlab version is different from R2014b, please modifly both MakeFiles
(in maslib and tetgen) to reflect the correct version. 

-----------------------
2.3 VERIFYING THE BUILD
-----------------------

Run /GMM-FEM/Scripts/fem_only_test.m to perform non-rigid FEM-based registration.

-------------------
2.4 TROUBLESHOOTING
-------------------

If you get an error regarding find during Tetgen and Maslib, it could be that the
Windows find.exe is being called instead of Linux style find.exe in MinGW. One
workaround is to make sure the Linux style is placed at the beginning of %PATH%
in environment variables.