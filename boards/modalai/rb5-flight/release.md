# Release procedure

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
