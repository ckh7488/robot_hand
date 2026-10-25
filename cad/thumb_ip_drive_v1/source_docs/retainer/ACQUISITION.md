# XL330 metal-horn central retainer: acquisition evidence

Reviewed 2026-10-25. Research only; no order, contact, hardware operation or CAD modification.

## What can actually be sourced

| Official item | Published contents relevant to this question | Conclusion |
|---|---|---|
| HNX330-N101 Set, 903-0314-000 | One metal horn; six PHS M2x4 frame screws | The central retainer is not listed as a separate included item. Do not assume this kit supplies it. |
| XL330-M077-T, 902-0162-000 | Motor, cable, six PHS M2x6 TAP and ten PHS M2x8 TAP | The package list does not identify the installed central horn screw or promise a spare BHS M2.6x6. It therefore does not establish which central screw can be reused for the metal horn. |
| FPX330-H101 4PCS SET, 903-0302-000 | Eight BHS M2.6x6 TAP screws, described for idlers, in addition to four frames/idlers/caps and frame fasteners | A concrete official retail package for the named M2.6x6 TAP family. This is the strongest acquisition route found. It is not yet proof that its screw exactly matches the metal-horn drawing variant. |

Sources: [metal-horn package](https://en.robotis.com/shop_en/item.php?it_id=903-0314-000), [XL330 package](https://en.robotis.com/shop_en/item.php?it_id=902-0162-000), [H101 package](https://en.robotis.com/shop_en/item.php?it_id=903-0302-000).

## Remaining identity and geometry gap

The existing official HNX330 assembly illustration labels the XL330 retainer `BHS_M2_6X06_TAP_K`; the XC330 alternative is `BHS_M2_5X06_NYLOK_K` and must not be substituted. Its file remains unchanged at `../hnx330_n101_assembly.png` (SHA256 6b0d79ef9ae1ba04a15c03f13ec98f8c32afd1b3c66416007cb2a223f9258c5a).

The H101 English package table says BHS M2.6x6 TAP, but its official assembly image and [Japanese official table](https://e-shop.robotis.co.jp/product.php?id=437) say BTS M2.6x6 TAP. The suffix K is omitted. No official equivalence statement or separate screw SKU was found in this bounded search. Therefore H101 is a documented supply lead, not a verified identical replacement. The manufacturer drawing or supplied physical part must resolve this naming inconsistency before an exact fit claim.

No dimensioned drawing of the precise central screw head, recess/driver, thread/pitch, taper, material, or tolerance was established. The official H101 STEP has one PRODUCT named DC15_A01_HINGE_1; it is frame geometry, not a central-screw model. The downloaded companion PDF is retained as a source, not treated as verified screw geometry. No screw CAD was generated.

The [BNS-30](https://robotis.us/products/bioloid-premium-bolt-nut-set-bns-30) list contains a different PHS M2.6x8 TAP K; [BNS-10](https://robotis.us/products/bolt-nut-set-bns-10) does not list the target. The official [screw category](https://robotis.us/collections/nuts-screws) did not identify an exact standalone target SKU. This is a search finding, not proof that such an item cannot be supplied.

## Consequence for the bench assembly

Use the official HNX330-N101 metal-horn arrangement: remove the plastic horn, fit one metal horn, secure it with the verified XL330 central retainer, then attach the separate spool with the four PCD12 M2 fasteners. This is a proposed assembly sequence; the central screw/tool access and installation are not validated. Keep `central_retention_complete=false` until the actual retainer, driver clearance and engagement are resolved. Do not shorten or rename the legacy M2.6x8 CAD to impersonate the missing screw. Do not substitute a generic machine M2.6x6 solely from nominal diameter/length.

`download_manifest.json` records every new official download URL, UTC retrieval time, byte count and SHA256. Frozen motor/source files were unchanged.