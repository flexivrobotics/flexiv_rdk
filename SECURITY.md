# Security Policy

## Reporting a vulnerability

Please report suspected security vulnerabilities in Flexiv RDK privately by email to
**security@flexiv.com**. Do not open a public GitHub issue, pull request or discussion for them.

Where possible, include:

* the affected RDK version(s), platform, and robot software version;
* a description of the issue and its impact;
* steps or a proof of concept to reproduce it;
* any suggested fix or mitigation.

## Coordinated vulnerability disclosure

We follow a coordinated disclosure process:

1. **Acknowledgement:** we acknowledge receipt of your report within 3 business days.
2. **Assessment:** we confirm whether the issue is a vulnerability, assess its severity, and keep
   you informed of our progress, normally at least every 2 weeks.
3. **Fix and advisory:** we aim to release a fix within 90 days of the report, sooner for critical
   issues. Fixed vulnerabilities are announced in the release notes and, where appropriate, in a
   GitHub security advisory with a CVE identifier.
4. **Disclosure:** please do not disclose the issue publicly until a fix has been released or 90
   days have passed since the report, whichever comes first. If we need more time, we will explain
   why and agree on a new date with you.
5. **Credit:** with your permission, we credit you in the advisory and release notes.

## Supported versions

Security fixes are provided for the latest minor release of this major version line.

| Version   | Supported                                                      |
| --------- | -------------------------------------------------------------- |
| 1.10.x    | Yes                                                            |
| < 1.10.0  | No, once 1.10.0 is released. Please upgrade to the latest 1.10.x |

Fixes are released as new patch or rebuild versions (see [RELEASING.md](RELEASING.md)). Published
release assets are never modified.

## Release signing key

Release tags, and the `SHA256SUMS` file published with every release, are signed with the Flexiv
RDK release signing key. See [RELEASING.md](RELEASING.md#verifying-a-release) for how to verify a
release.

* Public key: [cmake/flexiv_release_key.asc](cmake/flexiv_release_key.asc)
* Fingerprint: `TODO: to be published together with the release signing key`

> **TODO:** the release signing key has not been published yet. Until it is, the key file and the
> fingerprint above are placeholders.

The fingerprint is also set as `FLEXIV_RDK_RELEASE_KEY_FINGERPRINT` in
[CMakeLists.txt](CMakeLists.txt). Always compare the fingerprint of the key you import against the
one published here.

## Deployment security

RDK controls physical machinery. Please read this section before connecting RDK to a robot.

### The RDK-robot link is not authenticated or encrypted

RDK communicates with the robot over DDS (RTPS over UDP). In the current release this link is
**neither authenticated nor encrypted**:

* Any host that can reach the robot's network can discover the robot, read its states and, in
  principle, send it commands.
* All traffic, including robot states, commands and credentials, can be read and modified by
  anyone on the network path.

DDS Security (authentication, access control and encryption of the RDK-robot link) is planned for
a future release.

### `network_interface_whitelist` is not access control

The `network_interface_whitelist` argument of `rdk::Robot` only selects which **local** network
interface(s) of the computer running RDK are used to find and talk to the robot. It does not
restrict which remote hosts can talk to the robot or to RDK, and it provides no security.

### Isolate the robot network

Until DDS Security is available, the network must provide the protection:

* Connect the robot to a dedicated network: a dedicated NIC on the RDK computer, or a dedicated
  VLAN, with the robot and the RDK computer as its only hosts where possible.
* Do not connect untrusted hosts to that network segment, and do not bridge or route it to office,
  guest or internet-facing networks.
* Firewall the DDS UDP ports (the RTPS port range, 7400 and above with the default port mapping)
  between the robot network and every other network.
* Keep the operating system of the RDK computer up to date and restrict who can log in to it.

### Safety settings password

The password passed to `rdk::Safety(robot, password)` is the robot's **Elements admin password**,
and it is verified by the robot. It is only needed to change the safety settings:
`rdk::Safety(robot)` reads them without any password. Until DDS Security is available the password
travels over the network **in plaintext**, so:

* use `rdk::Safety(robot)` when only reading the safety settings, so that no password is sent;
* only use `rdk::Safety(robot, password)` on an isolated robot network as described above;
* do not reuse the Elements admin password for anything else;
* change it if it may have been exposed.
