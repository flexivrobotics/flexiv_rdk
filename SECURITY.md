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

| Version  | Supported                                                       |
| -------- | --------------------------------------------------------------- |
| 2.2.x    | Yes                                                             |
| < 2.2.0  | No, once 2.2.0 is released. Please upgrade to the latest 2.2.x  |

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

RDK communicates with the robot over Zenoh, discovering it by UDP multicast scouting (by default on
`224.0.0.224:7446`) and then exchanging data over direct TCP and UDP sessions. In the current
release this link is **neither authenticated nor encrypted**:

* Any host that can reach the robot's network can discover the robot, read its states and, in
  principle, send it commands.
* All traffic, including robot states, commands and credentials, can be read and modified by
  anyone on the network path.

The robot does apply safeguards of its own, e.g. it only accepts motion commands from the one RDK
client that holds direct control, and it verifies the credential of every safety setting request.
These limit what an accidental or misbehaving client can do, but they are not a substitute for
authentication and encryption of the link, which are planned for a future release.

### Isolate the robot network

Until the link is secured, the network must provide the protection:

* Connect the robot to a dedicated network: a dedicated NIC on the RDK computer, or a dedicated
  VLAN, with the robot and the RDK computer as its only hosts where possible.
* Do not connect untrusted hosts to that network segment, and do not bridge or route it to office,
  guest or internet-facing networks.
* Firewall the Zenoh traffic (the multicast scouting port and the TCP/UDP sessions between the
  robot and the RDK computer) between the robot network and every other network.
* Keep the operating system of the RDK computer up to date and restrict who can log in to it.

### Safety settings password

The password passed to `rdk::Safety` is the administrator password of the robot's user authority,
as set in Flexiv Elements, and it is verified by the robot. Until the link is encrypted it travels
over the network **in plaintext**, so:

* only use `rdk::Safety` on an isolated robot network as described above;
* do not reuse the administrator password for anything else;
* change it if it may have been exposed.
