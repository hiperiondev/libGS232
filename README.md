<div align="center">

  <img src="assets/logo.png" alt="logo" width="200" height="auto" />
  <h1>GS-232 library</h1>
  
  <p>
    Library for Yaesu Antenna Rotator GS-232 A and B protocol server 
  </p>

<br />
</div>

<!-- Table of Contents -->
# :notebook_with_decorative_cover: Table of Contents

- [About the Project](#star2-about-the-project)
  * [Features](#dart-features)
- [Usage](#eyes-usage)
- [License](#warning-license)
- [Contact](#handshake-contact)

<!-- About the Project -->
## :star2: About the Project

<!-- Features -->
### :dart: Features

- Complete Yaesu Antenna Rotator GS-232 A and B protocol

<!-- Usage -->
## :eyes: Usage

Initialize context:
```C
uint8_t gs232_init(gs232_t **ctx);
```
Destroy context:
```C
uint8_t gs232_deinit(gs232_t **ctx);
```
Parse received command buffer
```C
uint8_t gs232_parse_command(gs232_t **ctx, char *buffer, uint32_t buffer_len);
```
Create return string for parsed command buffer
```C
uint8_t gs232_return_string(gs232_t *ctx, uint8_t command, char *ret_str);
```

<!-- Roadmap -->
## :compass: Roadmap

* [ ] Complete tests

<!-- License -->
## :warning: License

Distributed under the MIT License. See LICENSE for more information.


<!-- Contact -->
## :handshake: Contact

Emiliano Augusto Gonzalez - egonzalez.hiperion@gmail.com

Project Link: [https://github.com/hiperiondev/libGS232/](https://github.com/hiperiondev/libGS232/)
