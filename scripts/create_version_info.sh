#!/bin/bash
#
# from http://stackoverflow.com/questions/3442874/in-git-how-can-i-write-the-current-commit-hash-to-a-file-in-the-same-commit

version=$(git describe)

commit=$(git log -1 --pretty="%H%n%ci") # hash \n date
commit_hash=$(echo "$commit" | head -1)
commit_date=$(echo "$commit" | head -2 | tail -1) # 2010-12-28 05:16:23 +0300

branch_name=$(git symbolic-ref -q HEAD) # http://stackoverflow.com/questions/1593051/#1593487
branch_name=${branch_name##refs/heads/}
branch_name=${branch_name:-HEAD} # 'HEAD' indicates detached HEAD situation


dirname=$1
filename=$dirname/VersionInfo.h
tmpfile=/tmp/`basename $filename`
shift

if [ -e $filename ]; then
	cp $filename $tmpfile
else
	touch $tmpfile
fi

echo "/* This file was automatically generated during make process on `date` */" > $filename
echo "/* All changes will be overwritten! */" >> $filename
echo "" >> $filename
echo "#ifndef _VERSION_INFO__H" >> $filename
echo "#define _VERSION_INFO__H" >> $filename
echo "" >> $filename
echo "const char *versionStr = \"$version\";" >> $filename
echo "" >> $filename
echo "const char *buildDateStr = \"`date +'%F %T %z'`\";" >> $filename
echo "" >> $filename
echo "const char *commitHashStr = \"$commit_hash\";" >> $filename
echo "const char *commitDateStr = \"$commit_date\";" >> $filename
echo "" >> $filename
echo "const char *branchNameStr = \"$branch_name\";" >> $filename
echo "" >> $filename
echo "#endif /* _VERSION_INFO_H */" >> $filename

diff $filename $tmpfile > /dev/null 2> /dev/null 

# if the file changes, touch dependency files
# (it contains the time, so we have always to touch the files)
if [ $? -ne 0 ]; then
	for i in $*; do
		touch $dirname/$i
	done
fi

rm -f $tmpfile

