#/bin/bash

PROJECTS=(
motion_primitive_library
)

cmd=$1
if [ "$cmd" == "-h" ]; then
  echo "usage: ./subproj add"
  echo "   or: ./subproj pullall"
  echo "   or: ./subproj pushall"
else
  case $cmd in
    add )
      for project in "${PROJECTS[@]}"
      do
        repo=git@github.com:sikang/$project.git
        prefix=$project
        echo Adding "$project" to "$prefix"
        git subtree add --prefix $prefix $repo master --squash
      done
      ;;
    pullall )
      for project in "${PROJECTS[@]}"
      do
        repo=git@github.com:sikang/$project.git
        prefix=$project
        echo Updating "$project" in "$prefix"
        git subtree pull --prefix $prefix $repo master --squash
      done
      ;;
    pushall )
      for project in "${PROJECTS[@]}"
      do
        repo=git@github.com:sikang/$project.git
        prefix=$project
        echo Updating "$project" in "$prefix"
        git subtree push --prefix $prefix $repo master
      done
      ;;
    * )
      echo "./subproj $1: invalid command"
      ;;
  esac
fi
