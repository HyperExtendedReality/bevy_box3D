use std::{env, fs, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-env-changed=BOX3D_SOURCE_DIR");
    let source = env::var_os("BOX3D_SOURCE_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../box3d"));
    assert!(
        source.join("include/box3d/box3d.h").is_file(),
        "Box3D source missing; set BOX3D_SOURCE_DIR or place box3d beside this crate"
    );
    println!("cargo:rerun-if-changed={}", source.join("src").display());
    println!(
        "cargo:rerun-if-changed={}",
        source.join("include").display()
    );
    println!("cargo:rerun-if-changed=native/bridge.c");

    let mut files: Vec<_> = fs::read_dir(source.join("src"))
        .expect("read Box3D sources")
        .map(|entry| entry.expect("read Box3D source entry").path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "c"))
        .collect();
    files.sort();
    let msvc = env::var("CARGO_CFG_TARGET_ENV").as_deref() == Ok("msvc");
    let x86 = matches!(
        env::var("CARGO_CFG_TARGET_ARCH").as_deref(),
        Ok("x86_64" | "x86")
    );
    // Box3D's CMake builds its AVX2 solver with /arch:AVX2 on MSVC and picks
    // it at run time; MSVC refuses to compile that file without the flag.
    let (avx2, files): (Vec<_>, Vec<_>) = files
        .into_iter()
        .partition(|path| msvc && x86 && path.file_name().is_some_and(|name| name == "wide8.c"));
    let configure = |build: &mut cc::Build| {
        build
            .include(source.join("include"))
            .include(source.join("src"))
            .std("c17")
            .flag_if_supported("-ffp-contract=off");
        if msvc {
            build.flag("/fp:precise");
        } else {
            build.flag_if_supported("-pthread");
        }
    };
    if !avx2.is_empty() {
        let mut build = cc::Build::new();
        configure(&mut build);
        build.files(avx2).flag("/arch:AVX2");
        build.compile("bevy_box3d_native_avx2");
    }
    let mut build = cc::Build::new();
    configure(&mut build);
    build.files(files).file("native/bridge.c");
    build.compile("bevy_box3d_native");
    if env::var("CARGO_CFG_TARGET_FAMILY").as_deref() == Ok("unix") {
        println!("cargo:rustc-link-lib=m");
        println!("cargo:rustc-link-lib=pthread");
    }
}
