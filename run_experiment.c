#include "types.h"
#include "stat.h"
#include "user.h"
#include "fcntl.h"

#define NUM_RODADAS 5 // TODO: MUDAR PARA 30
#define CPU_BOUND_MIN 6
#define CPU_BOUND_MAX 14
#define TOTAL_PROC 20

#define INF 1000000      // Valor grande para representar infinito
#define NUM_VERTICES 100 // Número de vértices do grafo
#define NUM_ARESTAS 200  // Número de arestas

// Estrutura para representar uma aresta
struct Aresta
{
    int origem, destino, peso;
};

// Função simples para gerar números aleatórios
static unsigned long next = 1; // Semente para o gerador

int myrandom()
{
    next = next * 1103515245 + 12345;        // Geração de um número pseudo-aleatório
    return (unsigned)(next / 65536) % 32768; // Retorna um número no intervalo [0, 32767]
}

// Função para inicializar o grafo com arestas aleatórias
void inicializar_grafo(struct Aresta arestas[])
{
    for (int i = 0; i < NUM_ARESTAS; i++)
    {
        arestas[i].origem = myrandom() % NUM_VERTICES;
        arestas[i].destino = myrandom() % NUM_VERTICES;
        arestas[i].peso = 1 + myrandom() % 100; // Peso aleatório entre 1 e 100
    }
}

// Função para encontrar o vértice com a menor distância
int min_distancia(int dist[], int visitado[])
{
    int min = INF, min_index = -1;

    for (int v = 0; v < NUM_VERTICES; v++)
    {
        if (!visitado[v] && dist[v] <= min)
        {
            min = dist[v], min_index = v;
        }
    }
    return min_index;
}

// Implementação do algoritmo de Dijkstra
void dijkstra(struct Aresta arestas[], int origem)
{
    int dist[NUM_VERTICES];     // Distâncias mínimas do vértice de origem
    int visitado[NUM_VERTICES]; // Marca se o vértice foi visitado

    // Inicializa todas as distâncias como infinito e visitado como falso
    for (int i = 0; i < NUM_VERTICES; i++)
    {
        dist[i] = INF;
        visitado[i] = 0;
    }

    dist[origem] = 0; // A distância do vértice de origem para si é 0

    // Calcula o caminho mínimo para todos os vértices
    for (int count = 0; count < NUM_VERTICES - 1; count++)
    {
        int u = min_distancia(dist, visitado); // Pega o vértice com a menor distância

        visitado[u] = 1; // Marca o vértice como processado

        // Atualiza os valores das distâncias dos vizinhos do vértice u
        for (int i = 0; i < NUM_ARESTAS; i++)
        {
            if (arestas[i].origem == u && !visitado[arestas[i].destino])
            {
                int v = arestas[i].destino;
                int peso = arestas[i].peso;

                if (dist[u] != INF && dist[u] + peso < dist[v])
                {
                    dist[v] = dist[u] + peso;
                }
            }
        }
    }
}

// Função simulada CPU-bound
void cpu_bound_task()
{
    struct Aresta arestas[NUM_ARESTAS];

    // Inicializa o grafo com arestas aleatórias
    inicializar_grafo(arestas);

    // Executa o algoritmo de Dijkstra para calcular o caminho mínimo a partir do vértice 0
    dijkstra(arestas, 0);

    // Encerra o processo filho após concluir a tarefa
    exit();
}

// Função simulada IO-bound
void io_bound_task()
{
    int fd = open("io_file.txt", O_CREATE | O_RDWR);
    if (fd < 0)
    {
        printf(1, "Erro ao abrir o arquivo\n"); // DELETE
        exit();
    }

    // Escreve 100 vezes 100 caracteres aleatórios
    char buffer[101];
    for (int i = 0; i < 100; i++)
    {
        buffer[0] = '0' + (i / 100);
        buffer[1] = '0' + ((i / 10) % 10);
        buffer[2] = '0' + (i % 10);
        for (int j = 3; j < 100; j++)
        {
            buffer[j] = 'a' + (myrandom() % 26); // Gera caracteres aleatórios
        }
        buffer[100] = '\n';                // Adiciona nova linha na posição 100
        write(fd, buffer, sizeof(buffer)); // Escreve a linha no arquivo
    }
    close(fd);

    int fd_ = open("io_file.txt", O_CREATE | O_RDWR);
    close(fd_);

    unlink("io_file.txt");
    exit();
}

// Função principal para rodar o experimento
void run_experiment()
{
    // medidas vazão
    int tput_min = 0;
    int tput_max = 0;

    for (int rodada = 1; rodada <= NUM_RODADAS; rodada++)
    {
        // vazão
        int inicial_time = uptime();

        int x = 1;
        int y = 1;
        /*
        int x = CPU_BOUND_MIN + (myrandom() % (CPU_BOUND_MAX - CPU_BOUND_MIN + 1));
        int y = TOTAL_PROC - x;
        */

        printf(1, "Rodada %d: %d processos CPU-bound, %d processos IO-bound\n", rodada, x, y);

        // Fork para processos CPU-bound
        for (int i = 0; i < x; i++) // TODO: mudar para i < x
        {
            int pid = fork();
            if (pid == 0)
            {
                cpu_bound_task(); // Processo filho
            }
        }

        // Fork para processos IO-bound
        for (int i = 0; i < y; i++) // TODO mudar para i < y
        {
            int pid = fork();
            if (pid == 0)
            {
                io_bound_task(); // Processo filho
            }
        }

        // Aguarda os processos terminarem
        for (int i = 0; i < TOTAL_PROC; i++)
        {
            wait();
        }

        // Vazão:
        int end_time = uptime();
        int tput_rodada = end_time - inicial_time;

        if (tput_rodada < tput_min || rodada == 1)
        {
            tput_min = tput_rodada;
        }

        if (tput_rodada > tput_max || rodada == 1)
        {
            tput_max = tput_rodada;
        }
        int tput_norm = 0;
        if (tput_max != tput_min)
        { // Evita divisão por zero
            tput_norm = 100 - ((tput_rodada - tput_min) * 100 / (tput_max - tput_min));
        }
        else
        {
            tput_norm = 100;
        }

        // Coleta de métricas (simplificado)
        // Aqui você coletaria as métricas como latência de IO, justiça entre processos, etc.

        printf(1, "Rodada %d finalizada\n", rodada);
        printf(1, "vazao normalizada rodada: %d\n", tput_norm);
        printf(1, "vazao rodada: %d\nvazao max: %d\nvazao in: %d\n", tput_rodada, tput_max, tput_min);
    }

    exit();
}

int main(int argc, char *argv[])
{
    run_experiment(); // Executa o experimento
    exit();
}

// overhead: maloc e free,  focar escalonador, justica -> implementar em kernel