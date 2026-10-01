# Component Datasheet Index

Run `./fetch_datasheets.sh` to populate this directory with the PDFs. Only the index and the fetch script are tracked in git.

| Component                    | Document                | Local file                              | Source |
|------------------------------|-------------------------|-----------------------------------------|--------|
| ESP32-S3                     | Datasheet               | `ESP32-S3_Datasheet.pdf`                | https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf |
| ESP32-S3                     | Technical Reference Man.| `ESP32-S3_TRM.pdf`                      | https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf |
| Waveshare ESP32-S3 Zero      | Board documentation     | `Waveshare_ESP32-S3_Zero.html`          | https://www.waveshare.com/wiki/ESP32-S3-Zero |
| PCA9685                      | Datasheet (NXP)         | `PCA9685_Datasheet.pdf`                 | https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf |
| MG90S                        | Datasheet               | `MG90S_Datasheet.pdf`                   | https://components101.com/sites/default/files/component_datasheet/MG90S-Datasheet.pdf |

## Notes

- The Waveshare board page is HTML, not a PDF. The fetch script saves the rendered page.
- ESP32-S3 TRM is large (~2000 pages). Download on demand; not fetched by default.
