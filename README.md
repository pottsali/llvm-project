# Notes for Mups16

## Installing dependencies

```
sudo apt-get install ninja-build
```

## Configuring

```
cd ~/git/llvm-project/build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -DLLVM_ENABLE_PROJECTS=clang -D LLBM_TARGETS_TO_BUILD= -D LLVM_EXPERIMENTAL_TARGETS_TO_BUILD=Mups16 -DLLVM_OPTIMIZED_TABLEGEN=On ../llvm
```

## Building required binaries

```
cd ~/git/llvm-project/build
cmake --build . -j8 --target llvm-objdump llc

```

## Building test program

```
cd ~/git/llvm-project/build

# building the test program to IL
$ ./bin/clang -target mups16 -c ~/git/cpu/cpp/target/hello_world/main.c -emit-llvm

# compiling the IL to assembly
$ ./bin/llc -O0 -march=mups16 -relocation-model=static -filetype=asm main.bc -o main.s && cat main.s

# compiling the IL directly to ELF
$ ./bin/llc -O0 -march=mups16 -relocation-model=static -filetype=obj main.bc -o main.o

```
