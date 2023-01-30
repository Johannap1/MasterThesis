These are the latex files for creating MSc Literature and Thesis reports
using the novel TU Delft house style.


These files originally implemented style classes for LaTeX master
thesis reports at the faculty of Aerospace engineering of the Delft University of Technology.

The files have been changed subsequently by MSc students and
staff members of the Delft Center for Systems and Control.

The most recent version of the style file can be found
at the DCSC website on the page
http://www.dcsc.tudelft.nl/Education/index.html
or direcly as
http://www.dcsc.tudelft.nl/Education/MScTemplateLatex_new.zip

If you find errors in these files or if you have corrected
bugs, please contact the DCSC MSc coordinator Ton van den Boom (a.j.j.vandenboom@tudelft.nl) 
so that the templates can be updated.


There are 2 styles:
1. In the subdirectory <DCSC Literature Style>:
   for a literature study with style file mscLiterature.cls
   example file mscLiterature.tex 
   To compile this file (windows only), two DOS command file have been added:
   - make_bitmap_Literature.bat       (creates pdf directly from dvi files, no eps graphics allowed)
   - make_postscript_Literature.bat (creates pdf files via postcript, eps graphics allowed)
   Just run one of these files and a file mscLiterature.pdf will result
   all auxiliary (temporary files) are removed by running the file mrproperLatex.bat
2. In the subdirectory <DCSC Thesis Style>
   for an MSc thesis with style file mscThesis.cls
   example fil mscThesis.tex 
   To compile this file (windows only), two DOS command file have been added:
   - make_bitmap_Thesis.bat       (creates pdf directly from dvi files, no eps graphics allowed)
   - make_postscript_Thesis.bat (creates pdf files via postcript, eps graphics allowed)
   Just run one of these files and a file mscLiterature.pdf will result
   all auxiliary (temporary files) are removed by running the file mrproperLatex.bat


NOTE: The first page (Titlepage) is not be changed with respect to lay-out. Only the picture on the titlepage is optional 

NOTE (4-6-2012):
With the introduction of Miktex 2.9, an updated version of MakeIndex is used. This version no longer accepts absolute path names. In the past TechnikCenter passed absolute path names.
This expecially is necessary when creating nomenclature tables. Under build profiles the commandline arguments to be passed to MakeIndex need to be altered to "%tm" and a post processor needs to be added calling the makeindex executable with arguments: -s  nomencl.ist "%tm.nlo" -o "%tm.nls"   
http://comments.gmane.org/gmane.comp.tex.miktex/10259
http://www.simonsilk.com/content/simonsilk/2011-jun/latex-list-notations-nomenclature


================================================================

Updated: 
*October 20, 2009  by Steven Mulder and Peter Heuberger

*March 16, 2010 by Peter Heuberger (with thanks to various students). See cls files for details 

*March 25, 2010 by Peter Heuberger  (thanks to Gabriel Lopes)

*April 10, 2010 by Peter Heuberger  (thanks to Bart De Schutter, adapted for LINUX)

*June 9, 2010 by Peter Heuberger  (added info on \(re)newcommand;  no signature page as default setting; make sure autocompiling detects errors; thanks to Arjan Degenaar, Bart De Schutter)

*August 18, 2010 by Peter Heuberger (thanks to Christiaan Kreuzen). Repaired erroneous use of \large in signature page of mcsthesis.cls. Now it is possible to use fontsizes 10pt, 11pt and 12pt.

*January 7, 2011 by Peter Heuberger (thanks to Marco Heimensem). Improved the black title box on the cover. Now it's really black!

*October 24,27, 2011 by Peter Heuberger (thanks to Stefan van Loenhout). Resolved problems with TUD banner, and Matlab/C++ listings

*November 15, 2011 by Peter Heuberger (thanks to Walter Eikelenboom). Repaired error on front page on MScthesis (text fell out of black box

*June 4, 2012 by Peter Heuberger (Thanks to Thijs van Krimpen). Added extra information in the readme file on the build commands, because of the new version of Makeindex. No latex changes applied

*June 6, 2012 by Peter Heuberger. Replaced Peter Heuberger as coordinator by Ton van den Boom