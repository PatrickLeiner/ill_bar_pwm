/*
 * prototypes.h
 *
 *  Created on: Jan 20, 2016
 *      Author: pat
 */

#ifndef PROTOTYPES_H_
#define PROTOTYPES_H_

/*
 *  ======== task_ill ========
 *
 *  Rückgabetyp: void
 *  Übergabeparameter: void
 *  Beschreibung: Taks der das Illumination_click steuert und die Erhaltenen Werte in Value schreibt.
 *      Kommunikation findet über I2C statt.
 */
void task_ill();
/*
 *  ======== task_bar ========
 *
 *  Rückgabetyp: void
 *  Übergabeparameter: void
 *  Beschreibung: Taks der das Bargraph_click steuert und die Werte von Value liest und ausgibt.
 *      Kommunikation findet über SPI statt mit manuellem CS auf GPIO Pin 7 der GPIO Base M.
 */
void task_bar();
/*
 *  ======== pwm_fxn ========
 *
 *  Rückgabetyp: void
 *  Übergabeparameter: void
 *  Beschreibung: Funktion zur erzeugung eines PWM Signals auf beliebigem GPIO Pin.
 */
void pwm_fxn();
/*
 *  ======== value_check ========
 *
 *  Rückgabetyp: int
 *  Übergabeparameter: uint32_t value
 *  Beschreibung: Funktion um gemessene Licht Werte in Werte von 0-10 umzuwandeln.
 *      Werte 0-10 representieren jeweils eine LED des Bargraph_click.
 */
int value_conversion(uint32_t value);
/*
 *  ======== init_i2c ========
 *
 *  Rückgabetyp: void
 *  Übergabeparameter: void
 *  Beschreibung: Funktion um die GPIO Pin's des BoosterPack1 für I2C zu initialisieren.
 */
void init_i2c();
/*
 *  ======== init_spi ========
 *
 *  Rückgabetyp: void
 *  Übergabeparameter: void
 *  Beschreibung: Funktion um die GPIO Pin's des BoosterPack2 für SPI zu initialisieren.
 *      Konfigurieren der GPIO Pin's für manuelles ChipSelect.
 *      Reset des Registers des Bargraph_click.
 */
void init_spi();
/*
 *  ======== setup_i2c ========
 *
 *  Rückgabetyp: void
 *  Übergabeparameter: void
 *  Beschreibung: Funktion um I2C am BoosterPack1 zu aktivieren.
 */
void setup_i2c();
/*
 *  ======== setup_spi ========
 *
 *  Rückgabetyp: void
 *  Übergabeparameter: void
 *  Beschreibung: Funktion um SPI am BoosterPack2 zu aktivieren.
 */
void setup_spi();
/*
 *  ======== mailBox_create ========
 *
 *  Rückgabetyp: void
 *  Übergabeparameter: void
 *  Beschreibung: Funktion zum erstellen einer Mailbox.
 */
void mailBox_create();
/*
 *  ======== task_create ========
 *
 *  Rückgabetyp: void
 *  Übergabeparameter: void
 *  Beschreibung: Funktion zum erstellen der Tasks.
 *      Taks Prioritäten von 0-15 -> 15 ist höhste Priorität.
 *      Verfügbarer Stacksize für Tasks in Bytes.
 */
void task_create();
/*
 *  ======== semaphore_create ========
 *
 *  Rückgabetyp: void
 *  Übergabeparameter: void
 *  Beschreibung: Funktion zum erstellen einer Semaphore.
 *      Semaphore zum verhindern von zeitgleichem zugriff zweier Tasks auf eine geteilte Variable.
 */
void semaphore_create()


#endif /* PROTOTYPES_H_ */
