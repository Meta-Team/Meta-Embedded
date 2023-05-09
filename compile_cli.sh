repo_base=$(dirname $(realpath $0))
cd $repo_base
# rm -rf build
cmake -GNinja -Bbuild -DSDK_CLEAR_ENV=ON .
cmake --build build
