# Firefighter Robot

Robot autónomo programado en AVR Assembly para el microcontrolador ATmega2560.

## Descripción

Implementa una máquina de 4 estados para detectar y extinguir incendios de forma autónoma:

- **Seguimiento de línea** — sensores TCRT5000 para navegación
- **Detección de llama** — fotoresistores con umbral ADC
- **Identificación de base** — sensor de color TCS230
- **Extinción** — bomba de agua controlada por servo

## Archivos

- `src/main.asm` — lógica principal: motores, sensores, bomba, servo

## Hardware

- Microcontrolador: ATmega2560
- IDE: Microchip Studio
- Lenguaje: AVR Assembly