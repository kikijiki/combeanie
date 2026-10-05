---
id: faq
title: FAQ
sidebar_position: 2
---

# FAQ

## Is neural perception the default?

No. On the default path object evidence comes from the **classical** wrist camera pipeline
(`ColourDepthBackend`: colour + depth), not from a learned model, and not from simulator ground
truth — ground truth is demoted to `/perception/ground_truth/*` for the error evaluators, and the
overhead camera contributes obstacles only (its object pipeline stays off). Learned models are
future sidecars behind the same observation contracts.

## Does the reasoner improve success rate?

Not necessarily. It is off by default, advisory only, and measured mainly for safe
degradation / latency, not as a reliability booster.

## Why did my goal go to another robot?

Interactive `launch-*` recipes do not lease a ROS domain. `just demo` and benchmarks do.
If another baseline is running on the same domain, goals can land on the wrong composition.

## Where is the internal engineering documentation?

Kept privately, outside this repository. This Docusaurus site (`website/`, `just docs` /
`just docs-serve`) is the public-facing manual.
