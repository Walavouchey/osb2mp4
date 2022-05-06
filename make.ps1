$should_link = 0;

mkdir obj -erroraction ignore >$null 2>&1;
mkdir obj/subprojects -erroraction ignore >$null 2>&1;

# DEPENDENCIES

$OPENCV_INCLUDE = "C:/opencv/build/include/"; # your opencv include directory
$OPENCV_LINK = "C:/opencv/build/x64/vc15/lib/"; # your opencv build library directory
$OPENCV_LIB = "opencv_world451";

# GIFDEC

$gifdec_files = ls subprojects/gifdec/src/*.c;
mkdir obj/subprojects/gifdec -erroraction ignore >$null 2>&1;
ForEach ($file in $gifdec_files) {
    $base = $file.BaseName;
    $name = $file.Name;
    if (-not (Test-Path -Path obj/subprojects/gifdec/$base.o)) {
        echo "Compiling $name...";
        clang $file -std=c99 -I subprojects/gifdec/lib/ -O3 -c -o "obj/subprojects/gifdec/$base.o" -Wno-deprecated-declarations -D "_WIN32";
        if ($LASTEXITCODE -eq 0) { $should_link = 1; }
    }
}

# SOURCE

$source_files = ls src/*.cpp;
mkdir obj/lib -erroraction ignore >$null 2>&1;
ForEach ($file in $source_files) {
    $base = $file.BaseName;
    $name = $file.Name;
    if (-not (Test-Path -Path subprojects/lib/$base.o)) {
        echo "Compiling $name...";
        clang $file -std=c++17 -I lib -I subprojects/gifdec/lib -I $OPENCV_INCLUDE -O3 -fopenmp -c -o "obj/lib/$base.o" -Wno-unsequenced;
        if ($LASTEXITCODE -eq 0) { $should_link = 1; }
    }
}

# MAIN

$main_files = ls *.cpp;
$compiled_files = @(0) * $main_files.count;
$index = 0;
mkdir obj/main -erroraction ignore >$null 2>&1;
ForEach ($file in $main_files) {
    $base = $file.BaseName;
    $name = $file.Name;
    if (-not (Test-Path -Path obj/main/$base.o)) {
        echo "Compiling $name...";
        clang $file -std=c++17 -I lib -I subprojects/gifdec/lib -I $OPENCV_INCLUDE -O3 -fopenmp -c -o "obj/main/$base.o" -Wno-unsequenced;
        if ($LASTEXITCODE -eq 0) { $compiled_files[$index] = 1; }
    }
    $index++;
}

# LINK

mkdir bin -erroraction ignore >$null 2>&1;
$index = 0;
ForEach ($file in $main_files) {
    $base = $file.BaseName;
    $name = $file.Name;
    if ((-not (Test-Path -Path "bin/$base.exe")) -or $should_link -or $compiled_files[$index]) {
        echo "Linking $base.exe...";
        clang obj/subprojects/gifdec/*.o obj/lib/*.o obj/main/$base.o -L $OPENCV_LINK -l $OPENCV_LIB -o "bin/$base.exe" -fopenmp;
    }
    $index++;
}
