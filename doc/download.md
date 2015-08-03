Download TEB-Package					{#download}
===================

For Users
---------

The project source code is managed by a subversion control system (SVN).
Find the svn repository at https://svc.rst.e-technik.tu-dortmund.de/svn/TimedElasticBand/C++/.

To get the newest (experimental and maybe unstable) version checkout the *trunk* folder using:

    svn co https://svc.rst.e-technik.tu-dortmund.de/svn/TimedElasticBand/C++/trunk teb_package

Find versions tagged as stable or *working and tested with windows* in the *tags* folder. \n
They can be checked out in a similar way (**Warning**: Never commit changes to an existing tagged version, copy it to your own branch instead). \n
To check out a specific branch of the project, consider the *branches* folder.


For Developers
--------------

**Work on the trunk**

Checkout the trunk:

    svn co https://svc.rst.e-technik.tu-dortmund.de/svn/TimedElasticBand/C++/trunk teb_package

Modify or add code and check if you can compile the code successfully.
Check the status of your changes:

    svn st

Check files marked with a question mark, whether they should be added to the repository (use `svn add <path/filename>`). \n
**Never** add build files and temporary files to the repository!

Afterwards commit your changes to the trunk:

    svn ci -m "This should be a comprehensive commit message for the log"

Next time you continue working on the trunk, don't forget to call `svn up` to get the latest updates. \n
In case of conflicts, please resolve them by merging the code or contact the author of the previous (conflicting) commit!


**Create your own branch**


*Way 1:*

Create the branch directly on the server:

    svn cp https://svc.rst.e-technik.tu-dortmund.de/svn/TimedElasticBand/C++/trunk \
    https://svc.rst.e-technik.tu-dortmund.de/svn/TimedElasticBand/C++/branches/mybranch

Now checkout your newly created branch.
If you have no working copy of the project yet, use:
    
    svn co https://svc.rst.e-technik.tu-dortmund.de/svn/TimedElasticBand/C++/branches/mybranch teb_mybranch

If you already have a working copy (e.g. the trunk folder) call

    svn switch https://svc.rst.e-technik.tu-dortmund.de/svn/TimedElasticBand/C++/branches/mybranch

in the root of your trunk folder.


*Way 2:*

In case you checked out the complete project (including all branches and tags) using:

    svn co https://svc.rst.e-technik.tu-dortmund.de/svn/TimedElasticBand/C++/ teb_package

Now create a new branch while copying all project files (internally svn is smart enough to prevent double disk space usage):

    mkdir branches/mybranch
    svn add branches/mybranch
    svn cp trunk branches/mybranch  # Or copy a tag from the tags folder instead of the trunk to switch to the branch


**Synchronizing your branch with the trunk**

To merge the most recent updates from the trunk into your own branch do the following. \n
First check what files have been changed compared to your branch:

    svn st -u

Merge the diferences in your local working directory:

    svn merge https://svc.rst.e-technik.tu-dortmund.de/svn/TimedElasticBand/C++/trunk

Often it is recommended to add `--dry-run` to the argument list in order to check for possible conflicts.
See the remarks below on manually resolving conflicts.

Finally commit your merged code to your branch.


**Merging a branch into the trunk**

First synchronize your branch with the trunk and commit changes like mentioned before.

Now checkout the trunk to a dedicated directory. \n
Make sure that you do not have any local changes on the trunk (`svn up; svn st`).

Merge the (already synchronized) branch "mybranch" into the trunk using:

    svn merge --reintegrate https://svc.rst.e-technik.tu-dortmund.de/svn/TimedElasticBand/C++/branches/mybranch

Finally, commit your merged trunk to the server.

[Click here](http://svnbook.red-bean.com/en/1.7/svn.ref.svn.c.merge.html) and [here](http://svnbook.red-bean.com/en/1.7/svn.branchmerge.basicmerging.html)
for more information about merging.

**Information about your working directory**

Get information on your current revision and if you are working on the trunk or a branch using

    svn info


**Reverting local changes**

Undo changes that are not yet committed:

    svn revert <file>

Or undo everything (switch to the main directory first):

    svn revert --recursive .

**Correcting commit log messages**

If you want to correct a previously made commit message, call:

    svn propedit svn:log --revprop -r <REV-NR> https://svc.rst.e-technik.tu-dortmund.de/svn/TimedElasticBand/C++/

where `<REV-NR>` denotes the revision number (you can get revision numbers via `svn log`).

**Resolving conflicts**

If svn was not able to solve conflicts automatically, try to open the files marked as conflicted and searh for
lines starting with "<<<<<<<<" and ">>>>>>>>". Edit them manually and afterwards call

    svn resolved <file>


**Useful symbolic revision names**

For some revisions, special symbolic names can be utilized:
- HEAD: latest (youngest) revision on the server
- BASE: the revision your  checkout was last updated against
- COMMITTED: last revision a file or directory was changed
- PREV: the last revision the file or directory was changed immediatley before COMMITTED