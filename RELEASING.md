# Releasing Flexiv RDK

This document describes how RDK releases are tagged, published and signed, and how users can verify
what they download. For reporting vulnerabilities, see [SECURITY.md](SECURITY.md).

## Policy

1. **Signed, annotated tags.** Every release is tagged with an annotated, GPG-signed tag made with
   the Flexiv RDK release signing key:

       git tag -s vX.Y.Z -m "Flexiv RDK vX.Y.Z"
       git push origin vX.Y.Z

   A release tag is never moved, deleted or re-created once it has been pushed.

2. **Release assets are immutable.** Once a release is published, its assets (the prebuilt
   libraries, `SHA256SUMS` and `SHA256SUMS.asc`) are never replaced or re-uploaded. Any rebuild,
   even one with no source change, is published as a new version by bumping the fourth version
   component, e.g. `v2.2.0` -> `v2.2.0.1`, with a new tag and a new release.

3. **GitHub immutable releases.** "Immutable releases" is enabled in the repository settings
   (Settings > General > Releases), so that GitHub itself rejects changes to the tag and assets of a
   published release. Publish a release only after all of its assets have been uploaded to the
   draft.

4. **Signed checksums.** Every release publishes, next to its assets:
   * `SHA256SUMS`: the SHA-256 of every asset, in `sha256sum` format.
   * `SHA256SUMS.asc`: a detached, ASCII-armored GPG signature of `SHA256SUMS` made with the
     release signing key.

5. **Checked-in hashes.** The `lib/*.sha256` files of the release commit contain the SHA-256 of the
   corresponding assets, so that `CMakeLists.txt` can verify the download even without GPG.

## Release signing key

The public part of the release signing key is committed as
[cmake/flexiv_release_key.asc](cmake/flexiv_release_key.asc), and its fingerprint is set as
`FLEXIV_RDK_RELEASE_KEY_FINGERPRINT` in [CMakeLists.txt](CMakeLists.txt) and published in
[SECURITY.md](SECURITY.md#release-signing-key).

> **TODO:** the release signing key has not been published yet. Both files above still contain
> placeholders, so `-DFLEXIV_RDK_VERIFY_SIGNATURE=ON` fails on purpose until they are filled in.

The secret key is held only by the release maintainers and is never stored in CI.

## Release steps

1. Bump the version in `CMakeLists.txt` (`project(... VERSION ...)`) and `doc/Doxyfile.in`
   (`PROJECT_NUMBER`), and in the Python package.
2. Build the release libraries for every platform, and update `lib/*.sha256` with their SHA-256.
3. Generate and sign the checksum list over the exact files that will be uploaded, using
   `scripts/generate_release_checksums.sh` in the library source repository:

       export RDK_RELEASE_SIGNING_KEY=<release key fingerprint>
       bash scripts/generate_release_checksums.sh --output-dir <dir> <artifact>...

   The script refuses to run if `RDK_RELEASE_SIGNING_KEY` is not set, and checks the signature it
   just made.
4. Commit, then create and push the signed tag (see Policy 1).
5. Create a draft GitHub release for the tag, upload all libraries plus `SHA256SUMS` and
   `SHA256SUMS.asc`, then publish it.
6. Download the published assets again and verify them as described below.

## Verifying a release

Import the release signing key and check that its fingerprint matches the one published in
[SECURITY.md](SECURITY.md#release-signing-key):

    gpg --import cmake/flexiv_release_key.asc
    gpg --fingerprint <fingerprint>

Verify the tag:

    git verify-tag vX.Y.Z

Verify the checksum list and the downloaded assets (run in the directory holding the downloads):

    gpg --verify SHA256SUMS.asc SHA256SUMS
    sha256sum --check --ignore-missing SHA256SUMS

On macOS, use `shasum -a 256 --check SHA256SUMS` instead of `sha256sum`.

### Automatic verification at build time

`CMakeLists.txt` always verifies the downloaded library against the checked-in `lib/*.sha256` at
configure time, and checks it again right before installing it. To additionally verify it against
the signed `SHA256SUMS` of the same release, configure with:

    cmake .. -DFLEXIV_RDK_VERIFY_SIGNATURE=ON

This requires `gpg`. It downloads `SHA256SUMS` and `SHA256SUMS.asc`, verifies the signature with
only the committed release key (in a temporary keyring under the build directory), accepts only a
signature by the key with the pinned fingerprint, and checks that the downloaded library's SHA-256
is the one listed for its file name.
