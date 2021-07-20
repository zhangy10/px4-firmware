# Release procedure

- Add release notes to this document
- On master branch
- Bump package version number in debian/control
- Commit and push everything
- tag it and push tag
  - Tag format: vX.Y.Z-modalai-rb5-flight-<type>
    - Type v for dev
    - Type p for alpha
    - Type t for beta
    - Type rc for Release Candidate RC
    - Type r for Release
- make clean
- make default and qurt
- package
- deploy / validate
- post package to cloud bucket

# Releases

## 1.0.6alpha

- Added support for Spektrum RC

## 1.0.5alpha

- Support for new 9.1 based SLPI release

## 1.0.4alpha

- First functional release
