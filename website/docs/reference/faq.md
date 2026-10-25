---
id: faq
title: FAQ
sidebar_position: 2
---

# FAQ

## Is neural perception the default?

No. On the default path object evidence comes from the **classical** wrist camera pipeline
(`ColourDepthBackend`: colour + depth), not from a learned model, and not from simulator ground
truth — ground truth publishes on `/perception/ground_truth/*` for the error evaluators, and the
overhead camera contributes obstacles only (its object pipeline is off). The identity channel uses
a SigLIP 2 embedding as a shortlist, but it is not part of the default launch and never names a
product by itself; see [Registry and identity](/docs/perception/registry-and-identity).

## Can the system tell two look-alike products apart?

Through the identity channel, by label text: the word that differs (a flavour) must be read, with a
clear margin over the runner-up. An embedding alone cannot separate a product from an unregistered
look-alike, so it is a shortlist only. When the word cannot be read the answer is "needs human", not a
guess. See [Registry and identity](/docs/perception/registry-and-identity) and the measured
[Results](/docs/perception/results#identity-ladder).

## Does a product need a barcode to be identified?

No. The product registry keys every product by an internal product id; the GTIN is optional. A product
without one is named by label text, and the `barcode_enabled` switch removes the barcode channel from
every decision.

## Can the robot restock boxes?

No. Boxes are observed, counted in lanes and placed in the planning scene, but the executor refuses to
grasp or place one. See [Known boundaries](/docs/perception/#known-boundaries).

## Does the reasoner improve success rate?

Not necessarily. It is off by default, advisory only, and measured mainly for safe
degradation / latency, not as a reliability booster.

## Why did my goal go to another robot?

Interactive `launch-*` recipes do not lease a ROS domain. `just demo` and benchmarks do.
If another baseline is running on the same domain, goals can land on the wrong composition.

## Where is the internal engineering documentation?

Kept privately, outside this repository. This Docusaurus site (`website/`, `just docs` /
`just docs-serve`) is the public-facing manual.
