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
    let mut build = cc::Build::new();
    build
        .files(files)
        .file("native/bridge.c")
        .include(source.join("include"))
        .include(source.join("src"))
        .std("c17")
        .flag_if_supported("-ffp-contract=off");
    if env::var("CARGO_CFG_TARGET_ENV").as_deref() == Ok("msvc") {
        build.flag("/fp:precise");
    } else {
        build.flag_if_supported("-pthread");
    }
    build.compile("bevy_box3d_native");
    if env::var("CARGO_CFG_TARGET_FAMILY").as_deref() == Ok("unix") {
        println!("cargo:rustc-link-lib=m");
        println!("cargo:rustc-link-lib=pthread");
    }
}
