-----BEGIN PGP SIGNED MESSAGE-----
Hash: SHA1

Format: 1.0
Source: linux
Binary: linux-source-3.13.0, linux-doc, linux-headers-3.13.0-107, linux-libc-dev, linux-tools-common, linux-tools-3.13.0-107, linux-cloud-tools-common, linux-cloud-tools-3.13.0-107, linux-image-3.13.0-107-generic, linux-image-extra-3.13.0-107-generic, linux-headers-3.13.0-107-generic, linux-image-3.13.0-107-generic-dbgsym, linux-tools-3.13.0-107-generic, linux-cloud-tools-3.13.0-107-generic, linux-udebs-generic, linux-image-3.13.0-107-generic-lpae, linux-image-extra-3.13.0-107-generic-lpae, linux-headers-3.13.0-107-generic-lpae, linux-image-3.13.0-107-generic-lpae-dbgsym, linux-tools-3.13.0-107-generic-lpae, linux-cloud-tools-3.13.0-107-generic-lpae, linux-udebs-generic-lpae, linux-image-3.13.0-107-lowlatency, linux-image-extra-3.13.0-107-lowlatency, linux-headers-3.13.0-107-lowlatency, linux-image-3.13.0-107-lowlatency-dbgsym, linux-tools-3.13.0-107-lowlatency, linux-cloud-tools-3.13.0-107-lowlatency, linux-udebs-lowlatency, linux-image-3.13.0-107-powerpc-e500,
 linux-image-extra-3.13.0-107-powerpc-e500, linux-headers-3.13.0-107-powerpc-e500, linux-image-3.13.0-107-powerpc-e500-dbgsym, linux-tools-3.13.0-107-powerpc-e500, linux-cloud-tools-3.13.0-107-powerpc-e500, linux-udebs-powerpc-e500, linux-image-3.13.0-107-powerpc-e500mc, linux-image-extra-3.13.0-107-powerpc-e500mc, linux-headers-3.13.0-107-powerpc-e500mc, linux-image-3.13.0-107-powerpc-e500mc-dbgsym, linux-tools-3.13.0-107-powerpc-e500mc, linux-cloud-tools-3.13.0-107-powerpc-e500mc, linux-udebs-powerpc-e500mc, linux-image-3.13.0-107-powerpc-smp, linux-image-extra-3.13.0-107-powerpc-smp, linux-headers-3.13.0-107-powerpc-smp, linux-image-3.13.0-107-powerpc-smp-dbgsym, linux-tools-3.13.0-107-powerpc-smp, linux-cloud-tools-3.13.0-107-powerpc-smp, linux-udebs-powerpc-smp, linux-image-3.13.0-107-powerpc64-emb, linux-image-extra-3.13.0-107-powerpc64-emb, linux-headers-3.13.0-107-powerpc64-emb, linux-image-3.13.0-107-powerpc64-emb-dbgsym, linux-tools-3.13.0-107-powerpc64-emb,
 linux-cloud-tools-3.13.0-107-powerpc64-emb, linux-udebs-powerpc64-emb, linux-image-3.13.0-107-powerpc64-smp, linux-image-extra-3.13.0-107-powerpc64-smp, linux-headers-3.13.0-107-powerpc64-smp, linux-image-3.13.0-107-powerpc64-smp-dbgsym, linux-tools-3.13.0-107-powerpc64-smp, linux-cloud-tools-3.13.0-107-powerpc64-smp,
 linux-udebs-powerpc64-smp
Architecture: all i386 amd64 armhf arm64 x32 powerpc ppc64el
Version: 3.13.0-107.154
Maintainer: Ubuntu Kernel Team <kernel-team@lists.ubuntu.com>
Standards-Version: 3.9.4.0
Vcs-Git: http://kernel.ubuntu.com/git-repos/ubuntu/ubuntu-trusty.git
Build-Depends: debhelper (>= 5), cpio, module-init-tools, kernel-wedge (>= 2.24ubuntu1), makedumpfile [amd64 i386], libelf-dev, libnewt-dev, libiberty-dev, rsync, libdw-dev, libpci-dev, dpkg (>= 1.16.0~ubuntu4), pkg-config, flex, bison, libunwind8-dev, openssl, libaudit-dev, bc, python-dev, gawk, device-tree-compiler [powerpc], u-boot-tools [powerpc], libc6-dev-ppc64 [powerpc]
Build-Depends-Indep: xmlto, docbook-utils, ghostscript, transfig, bzip2, sharutils, asciidoc
Package-List: 
 linux-cloud-tools-3.13.0-107 deb devel optional
 linux-cloud-tools-3.13.0-107-generic deb devel optional
 linux-cloud-tools-3.13.0-107-generic-lpae deb devel optional
 linux-cloud-tools-3.13.0-107-lowlatency deb devel optional
 linux-cloud-tools-3.13.0-107-powerpc-e500 deb devel optional
 linux-cloud-tools-3.13.0-107-powerpc-e500mc deb devel optional
 linux-cloud-tools-3.13.0-107-powerpc-smp deb devel optional
 linux-cloud-tools-3.13.0-107-powerpc64-emb deb devel optional
 linux-cloud-tools-3.13.0-107-powerpc64-smp deb devel optional
 linux-cloud-tools-common deb kernel optional
 linux-doc deb doc optional
 linux-headers-3.13.0-107 deb devel optional
 linux-headers-3.13.0-107-generic deb devel optional
 linux-headers-3.13.0-107-generic-lpae deb devel optional
 linux-headers-3.13.0-107-lowlatency deb devel optional
 linux-headers-3.13.0-107-powerpc-e500 deb devel optional
 linux-headers-3.13.0-107-powerpc-e500mc deb devel optional
 linux-headers-3.13.0-107-powerpc-smp deb devel optional
 linux-headers-3.13.0-107-powerpc64-emb deb devel optional
 linux-headers-3.13.0-107-powerpc64-smp deb devel optional
 linux-image-3.13.0-107-generic deb kernel optional
 linux-image-3.13.0-107-generic-dbgsym deb devel optional
 linux-image-3.13.0-107-generic-lpae deb kernel optional
 linux-image-3.13.0-107-generic-lpae-dbgsym deb devel optional
 linux-image-3.13.0-107-lowlatency deb kernel optional
 linux-image-3.13.0-107-lowlatency-dbgsym deb devel optional
 linux-image-3.13.0-107-powerpc-e500 deb kernel optional
 linux-image-3.13.0-107-powerpc-e500-dbgsym deb devel optional
 linux-image-3.13.0-107-powerpc-e500mc deb kernel optional
 linux-image-3.13.0-107-powerpc-e500mc-dbgsym deb devel optional
 linux-image-3.13.0-107-powerpc-smp deb kernel optional
 linux-image-3.13.0-107-powerpc-smp-dbgsym deb devel optional
 linux-image-3.13.0-107-powerpc64-emb deb kernel optional
 linux-image-3.13.0-107-powerpc64-emb-dbgsym deb devel optional
 linux-image-3.13.0-107-powerpc64-smp deb kernel optional
 linux-image-3.13.0-107-powerpc64-smp-dbgsym deb devel optional
 linux-image-extra-3.13.0-107-generic deb kernel optional
 linux-image-extra-3.13.0-107-generic-lpae deb kernel optional
 linux-image-extra-3.13.0-107-lowlatency deb kernel optional
 linux-image-extra-3.13.0-107-powerpc-e500 deb kernel optional
 linux-image-extra-3.13.0-107-powerpc-e500mc deb kernel optional
 linux-image-extra-3.13.0-107-powerpc-smp deb kernel optional
 linux-image-extra-3.13.0-107-powerpc64-emb deb kernel optional
 linux-image-extra-3.13.0-107-powerpc64-smp deb kernel optional
 linux-libc-dev deb devel optional
 linux-source-3.13.0 deb devel optional
 linux-tools-3.13.0-107 deb devel optional
 linux-tools-3.13.0-107-generic deb devel optional
 linux-tools-3.13.0-107-generic-lpae deb devel optional
 linux-tools-3.13.0-107-lowlatency deb devel optional
 linux-tools-3.13.0-107-powerpc-e500 deb devel optional
 linux-tools-3.13.0-107-powerpc-e500mc deb devel optional
 linux-tools-3.13.0-107-powerpc-smp deb devel optional
 linux-tools-3.13.0-107-powerpc64-emb deb devel optional
 linux-tools-3.13.0-107-powerpc64-smp deb devel optional
 linux-tools-common deb kernel optional
 linux-udebs-generic udeb debian-installer optional
 linux-udebs-generic-lpae udeb debian-installer optional
 linux-udebs-lowlatency udeb debian-installer optional
 linux-udebs-powerpc-e500 udeb debian-installer optional
 linux-udebs-powerpc-e500mc udeb debian-installer optional
 linux-udebs-powerpc-smp udeb debian-installer optional
 linux-udebs-powerpc64-emb udeb debian-installer optional
 linux-udebs-powerpc64-smp udeb debian-installer optional
Checksums-Sha1: 
 769d3e9207f796560b56b363779290a544e2e5cc 116419243 linux_3.13.0.orig.tar.gz
 88b6b2d6a75d2786660bd9ab6e17ed873c602b77 9456513 linux_3.13.0-107.154.diff.gz
Checksums-Sha256: 
 073d6a589655031564407e349c86a316941fc26ef3444bb73a092b43a48347ec 116419243 linux_3.13.0.orig.tar.gz
 6dfc5516d37e099b0ac1aa9cc47ea8ce5452628b390c9a139219aca35e017601 9456513 linux_3.13.0-107.154.diff.gz
Files: 
 8c85f9d0962f2a9335028e4879b03343 116419243 linux_3.13.0.orig.tar.gz
 c27d7d25bf99b53540c1c3223c9dbcc1 9456513 linux_3.13.0-107.154.diff.gz
Testsuite: autopkgtest

-----BEGIN PGP SIGNATURE-----
Version: GnuPG v1

iQIcBAEBAgAGBQJYWDFKAAoJENt0rrj9ziT8MhgQAJ62dvBObHFBQKQsIq7BoBRL
LNoFOdOepW6eYQ9EQ8ALipH2+E+j8lv4hoZE0zuDxf4cPdrd17D4sTge93OPNj3E
lTPXlISciUke8d2+/xyhkSJnBkXYEi7XHMi82EjGnZHeD5Q6as7p5FPZltVQ3/vB
bnEBHyrq54QXsf4mgfwnlktagPCNjCzGwO1BCxM+WrBJPePQUy+0wigMk2zPPCn8
PpFpFMh39MFuDLvMzhAX4kGMa+Sd3bIIZ1eyrGxzsNWIeP7u4y5nnZmMnR7TZ3pH
NRaucVVu+b/34IHKR/M003sElfW2RCehGVeyUY59FnoDG0rncPHbQYSI1QvUUJnX
zahjX4W2ViSY+fFnYAAX1Xt7WdqXalapxS060BCsZOUDmgYUwoZ6RwyvA4IKK/+2
FdcyIJ2Tg3+mdb4H9G3mZn2YiOwgyGv8fYDgBttlnZJJmMFXEIJo3HQbkaAQwRFT
76F/X4/y5Tfp0wpY6OqetEG1pCgAy3PKr7bsAsF9lPJwjH93JY1EzvFvdozflbIU
oDzAKzbLvbAuQkBneiELHStsHdOjprsoIZG6CwKEuDNnc7dDAb49eaWgzF+ryvs+
KBdo8nkLm/6NRSO+I0UqhzzpXagjL92w3DcZ9BfP0VD2e/mLqFQzVqFSx+3My/JV
d9g2Sj+SwsSkS/0LVONf
=Cxyb
-----END PGP SIGNATURE-----
