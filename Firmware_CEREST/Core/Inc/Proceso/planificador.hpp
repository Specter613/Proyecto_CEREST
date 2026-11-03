/*
 * planificador.hpp
 *
 *  Created on: 13 sept 2026
 *      Author: specter0163
 */

#ifndef INC_PROCESO_PLANIFICADOR_HPP_
#define INC_PROCESO_PLANIFICADOR_HPP_

#include <cstdint>

enum class PrioridadTarea : uint8_t
{
    CRITICO = 0,
    ALTO    = 1,
    NORMAL  = 2,
    BAJO    = 3,
};

class Planificador
{
public:
    //Delta debe estar en microsegundos no en mili
    using TareaCallback = void(*)(void *context, uint32_t delta_t_us);
    static constexpr uint8_t MaximoTareas = 16;

    // --- Task se mueve AQUÍ, en public, ANTES de cualquier método ---
    struct Tarea
    {
        TareaCallback callback = nullptr;
        void *context = nullptr;
        uint32_t periodoUs = 0;
        uint32_t lastRunUs = 0;
        PrioridadTarea prioridad = PrioridadTarea::NORMAL;
        uint32_t duracionEsperada = 0; //Us microsegundos
        uint32_t cuentaExcedida = 0;
        uint32_t cuentaOmitida = 0;
        const char *name = "unnamed";
    };

    bool AgregarTarea(TareaCallback callback, void *context, uint32_t periodoUs,
                 PrioridadTarea prioridad = PrioridadTarea::NORMAL,
                 uint32_t duracionEsperada = 0, const char *name = "unnamed");

    uint8_t ObtenerCantidadTareas() const { return taskCount_; }
    const Tarea& ObtenerTarea(uint8_t index) const { return tasks_[index]; }

    void Inicio(uint32_t budgetUs = 0);

    uint32_t ObtenerCuentaExcedida(uint8_t taskIndex) const;
    uint32_t ObtenerCuentaOmitida(uint8_t taskIndex) const;

private:
    Tarea tasks_[MaximoTareas];
    uint8_t taskCount_ = 0;

    uint8_t executionOrder_[MaximoTareas];

    void ReconstruirOrdenEjecucion();
};



#endif /* INC_PROCESO_PLANIFICADOR_HPP_ */
