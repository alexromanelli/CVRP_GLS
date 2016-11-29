/*
 * main.cpp
 *
 *  Created on: 03/10/2016
 *      Author: romanelli
 */

#include <iostream>
#include <cstdio>
#include <fstream>
#include <string>
#include <cstdlib>
#include <vector>
#include <sstream>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <cmath>
#include <time.h>
#include <sys/time.h>

#include "CVRPsolver.h"
#include "ConectorMySQL.h"

int minNumCaminhoes = 0;
int numLocais = 0;
int capacidade;
double** coord;
int** distancias;
int distanciaMaxima;
int* demandas;
int indDeposito;

// trim from start
static inline std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
            std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}

void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (getline(ss, item, delim)) {
    	if (item != "")
    		elems.push_back(item);
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

double calcularDistancia(double x1, double y1, double x2, double y2) {
	double dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
	return dist;
}

int nint(double x) {
	return (int) (x + 0.5);
}

void lerCoordenadas(std::ifstream* arqInstancia, int numConsumidores, int** distancias, int* distanciaMaxima,
		double** coord) {
	std::string linha;
	for (int i = 0; i < numConsumidores; i++) {
		getline(*arqInstancia, linha);
		std::vector<std::string> snums = split(trim(linha), ' ');
		coord[i][0] = atof(snums[1].c_str());
		coord[i][1] = atof(snums[2].c_str());
	}
	*distanciaMaxima = 0;
	// calcular os pesos (distâncias entre os consumidores)
	for (int i = 0; i < numConsumidores - 1; i++) {
		distancias[i][i] = 0;
		for (int j = i + 1; j < numConsumidores; j++) {
			distancias[i][j] = nint(calcularDistancia(coord[i][0], coord[i][1],
					coord[j][0], coord[j][1]));
			distancias[j][i] = distancias[i][j];
			if (distancias[i][j] > *distanciaMaxima)
				*distanciaMaxima = distancias[i][j];
		}
	}
}

void lerDemandas(std::ifstream* arqInstancia, int numLocais, int* demandas) {
	std::string linha;
	for (int i = 0; i < numLocais; i++) {
		getline(*arqInstancia, linha);
		linha = trim(linha);
		demandas[i] = atoi(linha.substr(linha.find(" ")).c_str());
	}
}

void imprimirPesos(int numCidades, int** pesos) {
	for (int i = 0; i < numCidades; i++) {
		for (int j = 0; j < numCidades; j++) {
			std::printf(" %6d ", pesos[i][j]);
		}
		std::printf("\n");
	}
}

void imprimirSolucao(int** rota, CVRPsolver* cvrpSolver, int capacidade, int numLocais) {
	std::printf("\nRota encontrada: custo = %d\n", nint(cvrpSolver->calcularCustoSolucaoPorDistancias(rota)));
	std::printf("\n [ Capacidade = %d ]\n\n", capacidade);
	for (int i = 0; i < numLocais; i++) {
		if (rota[i][0] == 0)
			break;
		std::printf(" -> Caminhão %d:\n", i + 1);
		std::printf("   -> Locais atendidos....: %d\n", rota[i][0]);
		std::printf("   -> Carga...............: %d\n", rota[i][1]);
		std::printf("   -> Distância percorrida: %d\n", rota[i][2]);
		std::printf("   --> Rota:   0 ");
		for (int j = 0; j < rota[i][0]; j++) {
			std::printf(", %3d ", rota[i][3 + j]);
		}
		std::printf(",   0\n");
	}
	std::printf("\n");
}

void imprimirSolucao2(int** rota, CVRPsolver* cvrpSolver) {
	int numCaminhoes = 0;
	for (int i = 0; i < cvrpSolver->getNumLocais(); i++)
		if (rota[i][CVRPsolverConstantes::IND_NUM_LOCAIS] > 0)
			numCaminhoes++;
		else
			break;

	std::printf("%d\n", numCaminhoes);

	for (int i = 0; i < numCaminhoes; i++) {
		std::printf("0");
		for (int j = 0; j < rota[i][0]; j++) {
			std::printf(",%d", rota[i][3 + j]);
		}
		std::printf(",0\n");
	}
	std::printf("\n");
}

CVRPsolver* obterSolver(const char* nomeInstancia, double lambda, int numIteracoes,
		int iteracoesAteReiniciarPenalidades, bool imprimirAcompanhamento, bool penalidadesSimetricas,
		bool salvarProgressoExecucao) {
	CVRPsolver* cvrpSolver = NULL;

	double alfa = 0;
	if (lambda < 0) {
		alfa = -lambda;
		lambda = -1;
	}

	// abrir arquivo de instância
	std::string s = nomeInstancia;
	s = "instances/" + s + ".vrp";
	std::ifstream arqInstancia(s.c_str());

	// processar arquivo de instância
	if (arqInstancia.is_open()) {
		std::string linha = "";
		int estado = 0;
		while (getline(arqInstancia, linha)) {
			switch (estado) {
			case 0: // procurando número mínimo de caminhões
				if (linha.substr(0, 7) == "COMMENT") {
					// exemplo: "COMMENT : (Augerat et al, Min no of trucks: 5, Optimal value: 784)"
					int posInicioNumCaminhoes = linha.find("Min no of trucks: ") + 18;
					int posFimNumCaminhoes = linha.find(", Optimal value");
					if (posFimNumCaminhoes == std::string::npos) {
						posFimNumCaminhoes = linha.find(", Best value");
					}
					int count = posFimNumCaminhoes - posInicioNumCaminhoes;
					std::string sMinNumCaminhoes = linha.substr(posInicioNumCaminhoes, count);
					minNumCaminhoes = atoi(sMinNumCaminhoes.c_str());
					estado = 1;
				}
				break;
			case 1: // procurando número de consumidores (dimensão)
				if (linha.substr(0, 9) == "DIMENSION") {
					std::string sDim = linha.substr(12);
					numLocais = atoi(sDim.c_str());
					distancias = new int*[numLocais];
					for (int i = 0; i < numLocais; i++)
						distancias[i] = new int[numLocais];
					demandas = new int[numLocais];
					estado = 2;
				}
				break;
			case 2: // procurando capacidade
				if (linha.substr(0, 8) == "CAPACITY") {
					std::string sCap = linha.substr(11);
					capacidade = atoi(sCap.c_str());
					estado = 3;
				}
				break;
			case 3: // procurando seção de coordenadas
				if (linha.substr(0, 18) == "NODE_COORD_SECTION") {
					coord = new double*[numLocais];
					for (int i = 0; i < numLocais; i++)
						coord[i] = new double[2]; // 2 : lat, lon
					lerCoordenadas(&arqInstancia, numLocais, distancias, &distanciaMaxima, coord);
					estado = 4;
				}
				break;
			case 4: // procurando seção de demandas
				if (linha.substr(0, 14) == "DEMAND_SECTION") {
					lerDemandas(&arqInstancia, numLocais, demandas);
					estado = 5;
				}
				break;
			case 5: // procurando depósito
				if (linha.substr(0, 13) == "DEPOT_SECTION") {
					getline(arqInstancia, linha);
					indDeposito = atoi(linha.c_str()) - 1;
					estado = 6;
				}
				break;
			case 6: // nada mais a fazer;
				break;
			}
		}

		// fechar arquivo de instância
		arqInstancia.close();

		cvrpSolver = new CVRPsolver(nomeInstancia, numLocais, indDeposito, minNumCaminhoes,
				capacidade, distancias, demandas, lambda, distanciaMaxima, numIteracoes,
				iteracoesAteReiniciarPenalidades, CVRPsolverConstantes::OPCAO_MELHOR_APRIMORANTE,
				imprimirAcompanhamento, penalidadesSimetricas, salvarProgressoExecucao);

		if (lambda == -1) {
			// calcular lambda
//			int** solucaoInicial = cvrpSolver->gerarSolucaoAleatoria();
			int** solucaoInicial = cvrpSolver->gerarSolucaoGulosa();
			int** s = cvrpSolver->gerarSolucaoVazia();
			long int q1;
			int q2;
			cvrpSolver->iniciarPenalidades();
			bool melhoriaEmG;
			int** solucao = cvrpSolver->localSearch(solucaoInicial, s, &q1, &q2, &melhoriaEmG, false);
			int custo = cvrpSolver->calcularCustoSolucaoPorDistancias(solucao);
			lambda = alfa * custo / numLocais;
			cvrpSolver->setLambda(lambda);
		}
	}
	return cvrpSolver;
}

int main(int argc, char* argv[]) {
	if (argc == 10) {
		// ler parâmetros
		char* nomeInstancia = argv[1];
		std::string opcaoBuscaLocal = std::string(argv[2]);
		std::string opcaoAprimoramento = std::string(argv[3]);
		double lambda = atof(argv[4]);
		int numIteracoes = atoi(argv[5]);
		int iteracoesAteReiniciarPenalidades = atoi(argv[6]);
		bool imprimirAcompanhamento = argv[7][0] == 's' ? true : false;
		bool penalidadesSimetricas = argv[8][0] == 's' ? true : false;
		bool salvarProgressoExecucao = argv[9][0] == 's' ? true : false;

		CVRPsolver* cvrpSolver = obterSolver(nomeInstancia, lambda, numIteracoes,
				iteracoesAteReiniciarPenalidades, imprimirAcompanhamento, penalidadesSimetricas,
				salvarProgressoExecucao);
		if (cvrpSolver != NULL) {
			int** solucao = cvrpSolver->guidedLocalSearch(opcaoBuscaLocal, opcaoAprimoramento);
			if (!cvrpSolver->verificarCapacidades(solucao)) {
				std::cout << "Erro na solução" << std::endl;
			}

			double cpuTime = CVRPsolver::get_cpu_time();

			if (imprimirAcompanhamento) {
				std::cout << "Instância.....: " << nomeInstancia << std::endl;
				std::cout << "Custo.........: " << cvrpSolver->calcularCustoSolucaoPorDistancias(solucao) << std::endl;
				std::cout << "Tempo.........: " << cpuTime << std::endl;
				std::cout << "Parâmetros GLS:" << std::endl;
				std::cout << " - Lambda.............: " << cvrpSolver->getLambda() << std::endl;
				std::cout << " - Número de iterações: " << numIteracoes << std::endl;
				std::cout << "Solução:" << std::endl;
			} else {
				std::printf("\"%s\";%d;%d;%f;%f\n", nomeInstancia,
						cvrpSolver->calcularCustoSolucaoPorDistancias(solucao),
						numIteracoes, cvrpSolver->getLambda(), CVRPsolver::get_cpu_time());
				imprimirSolucao2(solucao, cvrpSolver);
			}
			if (imprimirAcompanhamento) {
				std::cout << "Deseja salvar resultado? (s/n): ";
				char opcao;
				std::cin >> opcao;
				if (opcao == 's') {
					if (cvrpSolver->verificarCapacidades(solucao)) {
						// gravar no banco
						ConectorMySQL* conectorMySQL = new ConectorMySQL();
						int idResultado = conectorMySQL->inserirResultado(nomeInstancia, capacidade, numLocais,
								cvrpSolver->calcularCustoSolucaoPorDistancias(solucao),
								cvrpSolver->obterNumCaminhoesSolucao(solucao), lambda, cvrpSolver->getLambda(),
								numIteracoes, opcaoBuscaLocal, opcaoAprimoramento, iteracoesAteReiniciarPenalidades,
								penalidadesSimetricas, ((int)(cpuTime * 1000)),
								cvrpSolver->obterQuantidadeSolucoesAvaliadas(), solucao, demandas, coord);

						if (salvarProgressoExecucao) {
							conectorMySQL->inserirProgressoExecucao(idResultado, cvrpSolver->getNumAgrupamentosProgressoExecucao(),
									numIteracoes, cvrpSolver->obterVetorCustoMelhorSolucao());
						}

						// imprimir na tela
						std::printf("\"%s\";%d;%d;%f;%f\n", nomeInstancia,
								cvrpSolver->calcularCustoSolucaoPorDistancias(solucao),
								numIteracoes, cvrpSolver->getLambda(), cpuTime);
						imprimirSolucao2(solucao, cvrpSolver);
					} else {
						std::printf("Instancia: %s, Lambda: %.3f, Iterações: %d -> ERRO NA SOLUÇÃO.",
								nomeInstancia, lambda, numIteracoes);
					}
				}
			}

		} else {
			printf("Erro ao abrir arquivo de instância.\n");
		}
	} else if (argc == 2) {
		std::string opcao = std::string(argv[1]);
		if (opcao == "t") {
			ConectorMySQL* conectorMySQL = new ConectorMySQL();
			std::string nomeInstancia;
			double lambda;
			int iteracoes;
			std::string opcaoBuscaLocal;
			std::string opcaoAprimoramento;
			int iteracoesAteReiniciarPenalidades;
			bool penalidadesSimetricas;
			bool salvarProgressoExecucao = true;
			while (conectorMySQL->buscarProximoTestePendente(&nomeInstancia, &lambda, &iteracoes,
					&opcaoBuscaLocal, &opcaoAprimoramento, &iteracoesAteReiniciarPenalidades,
					&penalidadesSimetricas)) {
				CVRPsolver* cvrpSolver = obterSolver(nomeInstancia.c_str(), lambda, iteracoes,
						iteracoesAteReiniciarPenalidades, false, penalidadesSimetricas, salvarProgressoExecucao);
				if (cvrpSolver != NULL) {
					int quantidadeTestesPorConfiguracao = 1;
					// executar testes
					for (int i = 0; i < quantidadeTestesPorConfiguracao; i++) {
						double cpuTimeAnterior = CVRPsolver::get_cpu_time();

						int** solucao = cvrpSolver->guidedLocalSearch(opcaoBuscaLocal, opcaoAprimoramento);

						double cpuTimeAtual = CVRPsolver::get_cpu_time();
						double cpuTime = cpuTimeAtual - cpuTimeAnterior;

						if (cvrpSolver->verificarCapacidades(solucao)) {
							// gravar no banco
							int idResultado = conectorMySQL->inserirResultado(nomeInstancia, capacidade, numLocais,
									cvrpSolver->calcularCustoSolucaoPorDistancias(solucao),
									cvrpSolver->obterNumCaminhoesSolucao(solucao), lambda, cvrpSolver->getLambda(),
									iteracoes, opcaoBuscaLocal, opcaoAprimoramento, iteracoesAteReiniciarPenalidades,
									penalidadesSimetricas, ((int)(cpuTime * 1000)),
									cvrpSolver->obterQuantidadeSolucoesAvaliadas(), solucao, demandas, coord);
							if (i == 0) {
								conectorMySQL->registrarTesteExecutado(nomeInstancia, lambda, iteracoes,
										opcaoBuscaLocal, opcaoAprimoramento, iteracoesAteReiniciarPenalidades,
										penalidadesSimetricas, idResultado);
							}
							if (salvarProgressoExecucao) {
								conectorMySQL->inserirProgressoExecucao(idResultado, cvrpSolver->getNumAgrupamentosProgressoExecucao(),
										iteracoes, cvrpSolver->obterVetorCustoMelhorSolucao());
							}

							// imprimir na tela
							std::printf("\"%s\";%d;%d;%f;%f\n", nomeInstancia.c_str(),
									cvrpSolver->calcularCustoSolucaoPorDistancias(solucao),
									iteracoes, cvrpSolver->getLambda(), cpuTime);
							imprimirSolucao2(solucao, cvrpSolver);
						} else {
							std::printf("Instancia: %s, Lambda: %.3f, Iterações: %d -> ERRO NA SOLUÇÃO.",
									nomeInstancia.c_str(), lambda, iteracoes);
						}
					}
				}
			}
			delete conectorMySQL;
		}
	} else {
		// sem os parâmetros, considerar opção de criação de testes
		printf("Cadastrar testes? (s=sim/n=não): ");
		std::string resposta;
		std::cin >> resposta;
		if (resposta == "s") {
			// gerar combinações de parâmetros de testes
			std::string nomesInstancias[4] = {
					"A-n45-k6", "A-n63-k10", "A-n80-k10", "B-n56-k7" };
			int numerosIteracoes[4] = { 130000, 170000, 800000, 200000 };
			int numParametrosLambda = 11;
			double lambdas[numParametrosLambda] = { -0.4, -0.38, -0.36, -0.34, -0.32, -0.3, -0.28, -0.26, -0.24,
					-0.22, -0.2 };
			ConectorMySQL* conectorMySQL = new ConectorMySQL();
			for (int indLambda = 1; indLambda <= 100; indLambda++) {
				for (int indInstancia = 0; indInstancia < 4; indInstancia++) {
					conectorMySQL->inserirTeste(nomesInstancias[indInstancia], -(indLambda / 100.0),
							numerosIteracoes[indInstancia], "FLS", "P", 0, false);
				}
			}
		}
	}

}




