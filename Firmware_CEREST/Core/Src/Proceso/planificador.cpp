#include "Proceso/planificador.hpp"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim5;
static inline uint32_t MicrosNow()
{
    return __HAL_TIM_GET_COUNTER(&htim5);
}

bool Planificador::AgregarTarea(TareaCallback callback, void *context, uint32_t periodoUs,
                         PrioridadTarea prioridad, uint32_t duracionEsperada, const char *name)
{
    if(taskCount_ >= MaximoTareas) return false;
    Tarea &t = tasks_[taskCount_];
    t.callback = callback;
    t.context = context;
    t.periodoUs = periodoUs;
    t.lastRunUs = MicrosNow();
    t.prioridad = prioridad;
    t.duracionEsperada = duracionEsperada;
    t.cuentaExcedida = 0;
    t.cuentaOmitida = 0;
    t.name = name;
    taskCount_++;
    return true;
}

/**
 * @brief Ejecuta una tarea individual: mide el delta real, avanza fase,
 * corre el callback y registra estadísticas. Común a los dos caminos
 * de ejecución (forzado CRÍTICO y con presupuesto).
 */
static void EjecutarTarea(Planificador::Tarea &t, uint32_t elapsedUs)
{
    // Corrección de fase: avanza por período, no salta al 'now' actual —
    // evita que el jitter de una vuelta se acumule en las siguientes.
    t.lastRunUs += t.periodoUs;

    // Protección contra congelamientos severos (más de 3 periodos de atraso):
    // reincide la fase en vez de intentar alcanzar el pasado.
    uint32_t ahora = MicrosNow();
    if ((ahora - t.lastRunUs) > (t.periodoUs * 3))
    {
        t.lastRunUs = ahora;
    }

    uint32_t taskStartUs = MicrosNow();
    t.callback(t.context, elapsedUs);
    uint32_t taskDurationUs = MicrosNow() - taskStartUs;

    if(t.duracionEsperada > 0 && taskDurationUs > t.duracionEsperada)
    {
        t.cuentaExcedida++;
    }
}

void Planificador::Inicio(uint32_t budgetUs)
{
    uint32_t now = MicrosNow();          // TODO en microsegundos, un solo reloj
    uint32_t startUs = now;
    bool trackBudget = (budgetUs > 0);

    // =====================================================================
    // PASO 1 — CRÍTICO siempre corre si está vencido, SIN pasar por el
    // presupuesto. Igual que TASK_PRIORITY_REALTIME en iNav: una tarea
    // crítica nunca se salta por falta de presupuesto, solo la limita
    // el hardware. Esto es lo que garantiza que Gyro/Motor jamás se
    // queden sin correr aunque el resto del sistema esté sobrecargado.
    // =====================================================================
    for(uint8_t i = 0; i < taskCount_; i++)
    {
        Tarea &t = tasks_[i];
        if(t.prioridad != PrioridadTarea::CRITICO) continue;

        uint32_t elapsed = now - t.lastRunUs;
        if(elapsed < t.periodoUs) continue;

        EjecutarTarea(t, elapsed);
    }

    // =====================================================================
    // PASO 2 — El resto, ordenado por qué tan vencidas están (relación
    // elapsed/periodo), NO por prioridad estática fija. Esto es lo que
    // evita la inanición: una tarea NORMAL que lleva mucho tiempo sin
    // correr (elapsed/periodo grande) compite de igual a igual contra
    // una ALTO que apenas se venció, en vez de esperar siempre detrás
    // de todo lo de mayor prioridad. Al mismo tiempo, en igualdad de
    // urgencia, la prioridad estática sigue desempatando.
    // =====================================================================
    uint8_t candidatos[MaximoTareas];
    uint8_t nCandidatos = 0;
    for(uint8_t i = 0; i < taskCount_; i++)
    {
        if(tasks_[i].prioridad == PrioridadTarea::CRITICO) continue; // ya corrieron arriba
        uint32_t elapsed = now - tasks_[i].lastRunUs;
        if(elapsed >= tasks_[i].periodoUs)
        {
            candidatos[nCandidatos++] = i;
        }
    }

    // Ordena candidatos por urgencia (elapsed/periodo) descendente,
    // desempatando por prioridad estática. Selection sort simple:
    // nCandidatos es pequeño (<16), el costo es insignificante.
    for(uint8_t i = 0; i < nCandidatos; i++)
    {
        uint8_t mejor = i;
        for(uint8_t j = i + 1; j < nCandidatos; j++)
        {
            Tarea &ta = tasks_[candidatos[mejor]];
            Tarea &tb = tasks_[candidatos[j]];
            float urgA = (float)(now - ta.lastRunUs) / (float)ta.periodoUs;
            float urgB = (float)(now - tb.lastRunUs) / (float)tb.periodoUs;
            bool tbEsMasUrgente = (urgB > urgA + 1e-6f) ||
                (urgB > urgA - 1e-6f && tb.prioridad < ta.prioridad);
            if(tbEsMasUrgente) mejor = j;
        }
        uint8_t tmp = candidatos[i]; candidatos[i] = candidatos[mejor]; candidatos[mejor] = tmp;
    }

    for(uint8_t k = 0; k < nCandidatos; k++)
    {
        Tarea &t = tasks_[candidatos[k]];
        uint32_t elapsed = now - t.lastRunUs;

        if(trackBudget)
        {
            uint32_t usedSoFar = MicrosNow() - startUs;
            if(usedSoFar >= budgetUs)
            {
                t.cuentaOmitida++;
                continue;   // sigue intentando los demás candidatos por si alguno
                            // ya no cupiera igual — pero como el presupuesto ya
                            // se agotó, todos los que restan también se omiten
            }
        }

        EjecutarTarea(t, elapsed);
    }
}

uint32_t Planificador::ObtenerCuentaExcedida(uint8_t taskIndex) const
{
    if(taskIndex >= taskCount_) return 0;
    return tasks_[taskIndex].cuentaExcedida;
}

uint32_t Planificador::ObtenerCuentaOmitida(uint8_t taskIndex) const
{
    if(taskIndex >= taskCount_) return 0;
    return tasks_[taskIndex].cuentaOmitida;
}