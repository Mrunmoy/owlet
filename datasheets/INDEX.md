# Component Datasheet Index

Run `./fetch_datasheets.sh` to populate this directory with the PDFs. Only this index, the README, `sources.env` and the fetch script are tracked in git.

| Component                    | Document                | Local file                              | Source |
|------------------------------|-------------------------|-----------------------------------------|--------|
| RP2040                       | Datasheet               | `RP2040_Datasheet.pdf`                  | https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf |
| Raspberry Pi Pico            | Board datasheet         | `Pico_Datasheet.pdf`                    | https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf |
| TMC2209                      | Product page and datasheet | (not fetched)                        | https://www.analog.com/en/products/tmc2209.html |

## Notes

- The TMC2209 datasheet is on Analog Devices' product page, which tends to refuse scripted downloads; fetch it by hand.
- The NEMA 17 pancake motors are generic; use the seller's sheet for coil resistance and rated current.
