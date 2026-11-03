/*
 * uart_dispatch.cpp
 *
 *  Created on: 28 jul 2026
 *      Author: specter0163
 */
#include "Proceso/uart_dispatch.hpp"

UartDispatch::Registro UartDispatch::rx_[kMaxManejadores];       uint8_t UartDispatch::contadorRx_ = 0;
UartDispatch::Registro UartDispatch::rxMitad_[kMaxManejadores];  uint8_t UartDispatch::contadorRxMitad_ = 0;
UartDispatch::Registro UartDispatch::error_[kMaxManejadores];    uint8_t UartDispatch::contadorError_ = 0;
UartDispatch::Registro UartDispatch::tx_[kMaxManejadores];       uint8_t UartDispatch::contadorTx_ = 0;

void UartDispatch::Registrar(UART_HandleTypeDef *huart, ManejadorUart m) {
    if (contadorRx_ < kMaxManejadores) { rx_[contadorRx_] = {huart, m}; contadorRx_++; }
}
void UartDispatch::RegistrarMitad(UART_HandleTypeDef *huart, ManejadorUart m) {
    if (contadorRxMitad_ < kMaxManejadores) { rxMitad_[contadorRxMitad_] = {huart, m}; contadorRxMitad_++; }
}
void UartDispatch::RegistrarError(UART_HandleTypeDef *huart, ManejadorUart m) {
    if (contadorError_ < kMaxManejadores) { error_[contadorError_] = {huart, m}; contadorError_++; }
}
void UartDispatch::RegistrarTx(UART_HandleTypeDef *huart, ManejadorUart m) {
    if (contadorTx_ < kMaxManejadores) { tx_[contadorTx_] = {huart, m}; contadorTx_++; }
}

void UartDispatch::ManejarRxCompleto(UART_HandleTypeDef *huart) {
    for (uint8_t i = 0; i < contadorRx_; i++) if (rx_[i].huart == huart) { rx_[i].manejador(huart); return; }
}
void UartDispatch::ManejarRxMitadCompleto(UART_HandleTypeDef *huart) {
    for (uint8_t i = 0; i < contadorRxMitad_; i++) if (rxMitad_[i].huart == huart) { rxMitad_[i].manejador(huart); return; }
}
void UartDispatch::ManejarError(UART_HandleTypeDef *huart) {
    // Ya NO puede tocar estado de TX — ese estado ni siquiera vive aquí.
    for (uint8_t i = 0; i < contadorError_; i++) if (error_[i].huart == huart) { error_[i].manejador(huart); return; }
}
void UartDispatch::ManejarTxCompleto(UART_HandleTypeDef *huart) {
    for (uint8_t i = 0; i < contadorTx_; i++) if (tx_[i].huart == huart) { tx_[i].manejador(huart); return; }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { UartDispatch::ManejarRxCompleto(huart); }
extern "C" void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) { UartDispatch::ManejarRxMitadCompleto(huart); }
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) { UartDispatch::ManejarError(huart); }
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { UartDispatch::ManejarTxCompleto(huart); }
