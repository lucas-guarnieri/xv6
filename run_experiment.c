#include "types.h"
#include "stat.h"
#include "user.h"
#include "fcntl.h"

#define NUM_RODADAS 30
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
static unsigned long next = 0; // Seed
int myrandom()
{
    if (next == 0)
    {
        next = getpid();
    }
    next = next * 1103515245 + 12345;
    return (unsigned)(next / 65536) % 32768;
}

// Função para inicializar o grafo com arestas aleatórias
void inicializar_grafo(struct Aresta *arestas, int num_arestas, int num_vertices)
{
    for (int i = 0; i < num_arestas; i++)
    {
        arestas[i].origem = myrandom() % num_vertices;
        arestas[i].destino = myrandom() % num_vertices;
        arestas[i].peso = 1 + myrandom() % 100;
    }
}

// Função para encontrar o vértice com a menor distância
int min_distancia(int dist[], int visitado[], int num_vertices)
{
    int min = INF, min_index = -1;

    for (int v = 0; v < num_vertices; v++)
    {
        if (!visitado[v] && dist[v] <= min)
        {
            min = dist[v], min_index = v;
        }
    }
    return min_index;
}

// Função auxiliar para imprimir o caminho a partir do array predecessor
void imprimir_caminho(int predecessor[], int vertice)
{
    if (predecessor[vertice] == -1)
    {
        printf(1, "%d ", vertice);
        return;
    }
    imprimir_caminho(predecessor, predecessor[vertice]);
    printf(1, "%d ", vertice);
}

// Implementação do algoritmo de Dijkstra
void dijkstra(struct Aresta *arestas, int origem, int num_vertices, int num_arestas)
{
    int dist[num_vertices];
    int visitado[num_vertices];
    // int predecessor[num_vertices];

    // Inicializa todas as distâncias como infinito e visitado como falso
    for (int i = 0; i < num_vertices; i++)
    {
        dist[i] = INF;
        visitado[i] = 0;
        // predecessor[i] = -1;
    }

    dist[origem] = 0;

    // Calcula o caminho mínimo para todos os vértices
    for (int count = 0; count < num_vertices - 1; count++)
    {
        int u = min_distancia(dist, visitado, num_vertices);

        visitado[u] = 1;

        for (int i = 0; i < num_arestas; i++)
        {
            if (arestas[i].origem == u && !visitado[arestas[i].destino])
            {
                int v = arestas[i].destino;
                int peso = arestas[i].peso;

                if (dist[u] != INF && dist[u] + peso < dist[v])
                {
                    dist[v] = dist[u] + peso;
                    // predecessor[v] = u;
                }
            }
        }
    }

    /*
    printf(1, "Caminhos a partir do vértice %d:\n", origem);
    for (int i = 0; i < num_vertices; i++)
    {
        if (i != origem && dist[i] != INF)
        {
            printf(1, "Vértice %d (Distância %d): ", i, dist[i]);
            imprimir_caminho(predecessor, i);
            printf(1, "\n");
        }
    }
    */
}
// Função simula CPU-bound
void cpu_bound_task(int num_vertices, int num_arestas, int write_fd)
{
    int ini = uptime();
    struct Aresta *arestas = (struct Aresta *)malloc(num_arestas * sizeof(struct Aresta));

    if (arestas == 0)
    {
        printf(1, "Erro ao alocar memória para as arestas\n");
        exit();
    }

    // Inicializa o grafo com arestas aleatórias
    inicializar_grafo(arestas, num_arestas, num_vertices);

    // Executa o algoritmo de Dijkstra para calcular o caminho mínimo a partir do vértice 0
    dijkstra(arestas, 0, num_vertices, num_arestas);

    free(arestas);
    int end = uptime();
    int overhead = end - ini;
    write(write_fd, &overhead, sizeof(overhead));

    exit();
}

// Função simulada IO-bound
void io_bound_task(int write_fd)
{
    int ini = uptime();
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
    int end = uptime();
    int eficiencia = end - ini;
    write(write_fd, &eficiencia, sizeof(eficiencia));
    exit();
}

// Função principal para rodar o experimento
void run_experiment()
{
    // medidas vazão
    int tput_min = 0;
    int tput_max = 0;
    int tput_sum = 0;

    // medidas justiça
    int jus_min = 0;
    int jus_max = 0;
    int jus_sum = 0;

    // medidas overhead
    int over_min = 0;
    int over_max = 0;
    int over_sum = 0;

    // medidas eficiência
    int efic_min = 0;
    int efic_max = 0;
    int efic_sum = 0;

    for (int rodada = 1; rodada <= NUM_RODADAS; rodada++)
    {
        // vazão
        int inicial_time = uptime();

        int x = CPU_BOUND_MIN + (myrandom() % (CPU_BOUND_MAX - CPU_BOUND_MIN + 1));
        int y = TOTAL_PROC - x;

        printf(1, "Rodada %d: %d processos CPU-bound, %d processos IO-bound\n", rodada, x, y);

        // Criação pipe para cpu-bound:
        int pipe_cpu[2];
        if (pipe(pipe_cpu) < 0)
        {
            printf(1, "Erro ao criar pipe\n");
            exit();
        }

        // Fork para processos CPU-bound
        for (int i = 0; i < x; i++)
        {
            int pid = fork();
            if (pid == 0)
            {
                close(pipe_cpu[0]);
                int num_vertices = 100 + (myrandom() % 101);            // Aleatoriamente entre 100 e 200
                int num_arestas = 50 + (myrandom() % 351);              // Aleatoriamente entre 50 e 400
                cpu_bound_task(num_vertices, num_arestas, pipe_cpu[1]); // Processo filho
                exit();                                                 // Termina o filho após executar a tarefa
            }
            else if (pid > 0)
            {
                wait();
            }
        }
        close(pipe_cpu[1]);
        // Coleta resultados dos processos CPU-bound
        int overhead_rodada = 0;
        for (int i = 0; i < x; i++)
        {
            int number;
            read(pipe_cpu[0], &number, sizeof(number)); // Lê do pipe
            overhead_rodada += number;
        }
        close(pipe_cpu[0]);

        // Criação pipe para io-bound:
        int pipe_io[2];
        if (pipe(pipe_io) < 0)
        {
            printf(1, "Erro ao criar pipe\n");
            exit();
        }

        // Fork para processos IO-bound
        for (int i = 0; i < y; i++)
        {
            int pid = fork();
            if (pid == 0)
            {
                close(pipe_io[0]);
                io_bound_task(pipe_io[1]); // Processo filho
                exit();
            }
            else if (pid > 0)
            {
                wait();
            }
        }
        close(pipe_io[1]);
        // Coleta resultados dos processos io-bound
        int eficiencia_io_rodada = 0;
        for (int i = 0; i < y; i++)
        {
            int number;
            read(pipe_io[0], &number, sizeof(number)); // Lê do pipe
            eficiencia_io_rodada += number;
        }
        close(pipe_io[0]);

        // Vazão:
        int end_time = uptime();
        int tput_rodada = end_time - inicial_time;
        tput_sum += tput_rodada;
        int tput_norm = 0;
        if (tput_rodada < tput_min || rodada == 1)
            tput_min = tput_rodada;

        if (tput_rodada > tput_max || rodada == 1)
            tput_max = tput_rodada;
        if (tput_max != tput_min)
        {
            int tput_temp = tput_sum / rodada;
            tput_norm = 100 - ((tput_temp - tput_min) * 100 / (tput_max - tput_min));
        }
        else
        {
            tput_norm = 100;
        }

        // Justiça:
        int justica = cps();
        int jus_norm = 0;
        jus_sum += justica;
        if (justica < jus_min || rodada == 1)
        {
            jus_min = justica;
        }
        if (justica > jus_max || rodada == 1)
        {
            jus_max = justica;
        }
        if (jus_max != jus_min)
        {
            int jus_temp = jus_sum / rodada;
            jus_norm = 100 - ((jus_temp - jus_min) * 100 / (jus_max - jus_min));
        }
        else
        {
            jus_norm = 100;
        }

        // Overhead:
        int over_norm = 0;
        over_sum += overhead_rodada;
        if (overhead_rodada < over_min || rodada == 1)
            over_min = overhead_rodada;
        if (overhead_rodada > over_max || rodada == 1)
            over_max = overhead_rodada;
        if (over_min != over_max)
        {
            int over_temp = over_sum / rodada;
            over_norm = 100 - ((over_temp - over_min) * 100 / (over_max - over_min));
        }
        else
        {
            over_norm = 100;
        }

        // Eficiência:
        int efic_norm = 0;
        efic_sum += eficiencia_io_rodada;
        if (eficiencia_io_rodada < efic_min || rodada == 1)
            efic_min = eficiencia_io_rodada;
        if (eficiencia_io_rodada > efic_max || rodada == 1)
            efic_max = eficiencia_io_rodada;
        if (efic_min != efic_max)
        {
            int efic_temp = efic_sum / rodada;
            efic_norm = 100 - ((efic_temp - efic_min) * 100 / (efic_max - efic_min));
        }
        else
        {
            efic_norm = 100;
        }
        int total_perf = ((25 * tput_norm) + (25 * jus_norm) + (25 * over_norm) + (25 * efic_norm)) / 100;
        printf(1, "DESEMPENHO MÉDIO:          %d\n", total_perf);
        printf(1, " -Vazão normalizada:       %d\n", tput_norm);
        printf(1, " -Justiça:                 %d\n", jus_norm);
        printf(1, " -Overhead normalizado:    %d\n", over_norm);
        printf(1, " -Eficiência normalizada:  %d\n", efic_norm);
        printf(1, "_____________________________");
        printf(1, "\n");
    }

    exit();
}

int main(int argc, char *argv[])
{
    run_experiment();
    exit();
}