# selfmake_k230_v2_supplement_train

- Base dataset: `selfmake_k230_v1_80_20`
- Train: 396 images (`238` base + `158` supplement)
- Validation: 59 unchanged base images
- Supplement: 112 positive images, 46 negative images, 467 objects
- Excluded: `wire_contact_026.jpg` and `wire_contact_027.jpg`
- Class: `0 = steel_ball`

All supplement images are train-only. The frozen difficult test set is not included.
Keeping the original validation split unchanged makes V5 and V6 metrics comparable.
