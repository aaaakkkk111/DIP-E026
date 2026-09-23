# Debug signing

The development keystore used for the locally installed APK is intentionally
not committed. Gradle falls back to the developer machine's standard debug key
when `routea-debug.keystore` is absent.

To install a locally rebuilt APK as an update over the APK in `releases/`, use
the same private signing key that produced that release. Otherwise uninstall
the installed debug application before installing a build signed by a new key.
