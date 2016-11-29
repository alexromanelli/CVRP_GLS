/*
 * CVRPsolver.cpp
 *
 *  Created on: 12/10/2016
 *      Author: romanelli
 */

#include "CVRPsolver.h"

#include <stdlib.h>
#include <cmath>
#include <time.h>
#include <limits>
#include <cfloat>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>

CVRPsolver::CVRPsolver(std::string ni, int nl, int id, int nmdc, int cap, int** dist, int* dem, double l, int dm, int nmi,
		int iarp, int oa, bool ia, bool ps, bool sddpde) {
	this->nomeInstancia = ni;
	this->numLocais = nl;
	this->indDeposito = id;
	this->numMinDeCaminhoes = nmdc;
	this->capacidade = cap;
	this->distancias = dist;
	this->distanciaMaxima = dm;
	this->demandas = dem;
	this->lambda = l;
	this->numMaxIteracoes = nmi;
	this->iteracoesAteReiniciarPenalidades = iarp;
	this->opcaoAprimorante = oa;
	this->solucoesAuxiliaresNulas = true;
	this->imprimirAcompanhamento = ia;
	this->penalidadesSimetricas = ps;
	this->salvarDadosDeProgressoDaExecucao = sddpde;

	iniciarMatrizDeVizinhosPorMenorDistancia();
}

CVRPsolver::~CVRPsolver() {
	// TODO Auto-generated destructor stub
}

int** CVRPsolver::resolver() {
//	return this->guidedLocalSearch();
	return 0;
}

int** CVRPsolver::gerarSolucaoVazia() {
	int** solucao = new int*[this->numLocais];
	for (int i = 0; i < this->numLocais; i++) {
		solucao[i] = new int[this->numLocais + 3];
		solucao[i][0] = 0; // número de locais visitados pelo caminhão i
		solucao[i][1] = 0; // carga do caminhão i
		solucao[i][2] = 0; // distância percorrida pelo caminhão i
		for (int j = 0; j < this->numLocais; j++) {
			solucao[i][3 + j] = -1;
		}
	}
	return solucao;
}

int** CVRPsolver::gerarSolucaoAleatoria() {
	int** solucao = this->gerarSolucaoVazia();

	bool* localAlocado = new bool[this->numLocais];
	localAlocado[0] = true; // depósito definido como alocado para evitar que seja usado depois
	for (int i = 1; i < this->numLocais; i++) {
		localAlocado[i] = false;
	}

	int caminhoesAlocados = this->numMinDeCaminhoes;

	srand(time(NULL));
	for (int i = 0; i < this->numLocais - 1; i++) {
		// escolhe aleatoriamente o local (se estiver localAlocado, encontra o próximo não localAlocado)
		int indLocal = rand() % this->numLocais;
		while (localAlocado[indLocal])
			indLocal = (indLocal + 1) % this->numLocais;
		localAlocado[indLocal] = true;

		// escolhe aleatoriamente um caminhão alocado (se não puder cumprir a demanda, encontra o próximo
		// que possa, se nenhum puder, aloca um novo caminhão)
		int indCaminhao = rand() % caminhoesAlocados;
		int numTentativas = 1;
		while (solucao[indCaminhao][1] + this->demandas[indLocal] > this->capacidade) {
			indCaminhao = (indCaminhao + 1) % caminhoesAlocados;
			numTentativas++;
			if (numTentativas > caminhoesAlocados) {
				caminhoesAlocados++;
				indCaminhao = caminhoesAlocados - 1;
				break;
			}
		}

		// incrementa número de locais visitados pelo caminhão
		solucao[indCaminhao][0]++;

		// soma à carga do caminhão a demanda do local acrescentado à rota
		solucao[indCaminhao][1] += this->demandas[indLocal];

		// soma à distância percorrida pelo caminhão a distância do último local visitado até o local acrescentado à rota
		int indUltimoLocalVisitado = solucao[indCaminhao][0] == 1 ?
				0 : // anterior foi o depósito
				solucao[indCaminhao][2 + solucao[indCaminhao][0] - 1];
		solucao[indCaminhao][2] += this->obterDistancia(indUltimoLocalVisitado, indLocal);

		// inclui o local visitado como o próximo do vetor de locais visitados pelo caminhão
		solucao[indCaminhao][2 + solucao[indCaminhao][0]] = indLocal;
	}

	// completar os totais de distâncias com as distâncias de retorno ao depósito
	for (int i = 0; i < caminhoesAlocados; i++) {
		int indUltimoLocalVisitado = solucao[i][2 + solucao[i][0]];
		int distancia = this->obterDistancia(0, indUltimoLocalVisitado);
		solucao[i][2] += distancia;
	}

	return solucao;
}

void CVRPsolver::executar2Opt(int** rotaOriginal, int custoRotaOriginal, double custoAumentadoRotaOriginal,
		int** rotaAlterada, int* custoRotaAlterada, double* custoAumentadoRotaAlterada,
		int caminhao, int indLocal1, int indLocal2) {
	//this->copiarRota(rotaOriginal, rotaAlterada);
	for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
		int quantidadeLocais = rotaOriginal[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
		if (quantidadeLocais == 0) {
			rotaAlterada[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
			break;
		}

		memcpy(rotaAlterada[indCaminhao], rotaOriginal[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
		if (rotaAlterada[indCaminhao][0] != rotaOriginal[indCaminhao][0])
			std::printf("Erro.\n");
	}

	// inverte ordem de indLocal1 até indLocal2
	for (int i = 0; i <= indLocal2 - indLocal1; i++) {
		rotaAlterada[caminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1 + i] =
				rotaOriginal[caminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2 - i];
	}

	// identifica vértices das posições indLocal1 e indLocal2 do caminhão
	int local1 = rotaOriginal[caminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1];
	int local2 = rotaOriginal[caminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2];

	// identifica vértices antecessor e sucessor
	int antecessorLocal1 = indLocal1 == 0 ? 0 :
			rotaOriginal[caminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1 - 1];
	int sucessorLocal2 = indLocal2 == rotaOriginal[caminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] - 1 ?
			0 : rotaOriginal[caminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2 + 1];

	// calcula custos das arestas removidas
	int custoArestasRemovidas = this->distancias[antecessorLocal1][local1] +
			this->distancias[local2][sucessorLocal2];

	// calcula custos aumentados das arestas removidas
//	double custoAumentadoArestasRemovidas = custoArestasRemovidas + this->lambda *
//			(this->penalidades[antecessorLocal1][local1] + this->penalidades[local2][sucessorLocal2]);

	// calcula custos das arestas incluídas
	int custoArestasIncluidas = this->distancias[antecessorLocal1][local2] +
			this->distancias[local1][sucessorLocal2];

	// calcula custos aumentados das arestas incluídas
//	double custoAumentadoArestasIncluidas = custoArestasIncluidas + this->lambda *
//			(this->penalidades[antecessorLocal1][local2] + this->penalidades[local1][sucessorLocal2]);

	// calcula custos da rota alterada
	*custoRotaAlterada = custoRotaOriginal - custoArestasRemovidas + custoArestasIncluidas;
//	*custoAumentadoRotaAlterada = custoAumentadoRotaOriginal - custoAumentadoArestasRemovidas +
//			custoAumentadoArestasIncluidas;
	int distanciaPercorridaCaminhaoOriginal = rotaOriginal[caminhao][CVRPsolverConstantes::IND_DIST_PERCORRIDA];
	int distanciaPercorridaCaminhaoAlterada = this->calcularCustoSolucaoPorCaminhao(rotaAlterada, caminhao);
	double termoRegularizacaoCaminhaoOriginal = this->calcularTermoRegularizacaoSolucaoPorCaminhao(rotaOriginal, caminhao);
	double termoRegularizacaoCaminhaoAlterada = this->calcularTermoRegularizacaoSolucaoPorCaminhao(rotaAlterada, caminhao);

	*custoAumentadoRotaAlterada = custoAumentadoRotaOriginal - distanciaPercorridaCaminhaoOriginal +
			distanciaPercorridaCaminhaoAlterada - termoRegularizacaoCaminhaoOriginal +
			termoRegularizacaoCaminhaoAlterada;

	// teste para correção de erro
//	if (*custoAumentadoRotaAlterada != this->calcularCustoAumentadoSolucaoPorDistancias(rotaAlterada))
//		printf("Erro\n");

	// atualiza custo da sub-rota do caminhão
	rotaAlterada[caminhao][CVRPsolverConstantes::IND_DIST_PERCORRIDA] +=
			custoArestasIncluidas - custoArestasRemovidas;
}

bool CVRPsolver::executarTroca(int** rotaOriginal, int custoRotaOriginal, double custoAumentadoRotaOriginal,
		int** rotaAlterada, int* custoRotaAlterada, double* custoAumentadoRotaAlterada,
		int indCaminhao1, int indLocal1, int indCaminhao2, int indLocal2) {
	// identifica vértices trocados
	int local1 = rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1];
	int local2 = rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2];

	// verifica viabilidade de troca
	if (indCaminhao1 != indCaminhao2 &&
			(rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_CARGA] - this->demandas[local1] + this->demandas[local2] > this->capacidade ||
			 rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_CARGA] - this->demandas[local2] + this->demandas[local1] > this->capacidade))
		return false; // inviável se carga do caminhão 1 ou carga do caminhão 2 superarem capacidade

	// procede alteração de rota...

	//this->copiarRota(rotaOriginal, rotaAlterada);
	for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
		int quantidadeLocais = rotaOriginal[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
		if (quantidadeLocais == 0) {
			rotaAlterada[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
			break;
		}

		memcpy(rotaAlterada[indCaminhao], rotaOriginal[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
		if (rotaAlterada[indCaminhao][0] != rotaOriginal[indCaminhao][0])
			std::printf("Erro.\n");
	}

	// troca locais
	rotaAlterada[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1] =
			rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2];
	rotaAlterada[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2] =
			rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1];

	if (indCaminhao1 != indCaminhao2 || (indCaminhao1 == indCaminhao2 && indLocal1 + 1 < indLocal2)) {
		// identifica vértices antecessores e sucessores
		int antecessorLocal1 = indLocal1 == 0 ? 0 :
				rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1 - 1];
		int antecessorLocal2 = indLocal2 == 0 ? 0 :
				rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2 - 1];
		int sucessorLocal1 = indLocal1 == rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_NUM_LOCAIS] - 1 ?
				0 : rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1 + 1];
		int sucessorLocal2 = indLocal2 == rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_NUM_LOCAIS] - 1 ?
				0 : rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2 + 1];

		// calcula custos das arestas removidas
		int custoArestasRemovidas1 = this->distancias[antecessorLocal1][local1] +
				this->distancias[local1][sucessorLocal1];
		int custoArestasRemovidas2 = this->distancias[antecessorLocal2][local2] +
				this->distancias[local2][sucessorLocal2];

		// calcula custos aumentados das arestas removidas
		double custoAumentadoArestasRemovidas1 = custoArestasRemovidas1 + this->lambda *
				(this->penalidades[antecessorLocal1][local1] + this->penalidades[local1][sucessorLocal1]);
		double custoAumentadoArestasRemovidas2 = custoArestasRemovidas2 + this->lambda *
				(this->penalidades[antecessorLocal2][local2] + this->penalidades[local2][sucessorLocal2]);

		// calcula custos das arestas incluídas
		int custoArestasIncluidas1 = this->distancias[antecessorLocal1][local2] +
				this->distancias[local2][sucessorLocal1];
		int custoArestasIncluidas2 = this->distancias[antecessorLocal2][local1] +
				this->distancias[local1][sucessorLocal2];

		// calcula custos aumentados das arestas incluídas
		double custoAumentadoArestasIncluidas1 = custoArestasIncluidas1 + this->lambda *
				(this->penalidades[antecessorLocal1][local2] + this->penalidades[local2][sucessorLocal1]);
		double custoAumentadoArestasIncluidas2 = custoArestasIncluidas2 + this->lambda *
				(this->penalidades[antecessorLocal2][local1] + this->penalidades[local1][sucessorLocal2]);

		// calcula custos da rota alterada
		*custoRotaAlterada = custoRotaOriginal - custoArestasRemovidas1 - custoArestasRemovidas2 +
				custoArestasIncluidas1 + custoArestasIncluidas2;
		*custoAumentadoRotaAlterada = custoAumentadoRotaOriginal - custoAumentadoArestasRemovidas1 -
				custoAumentadoArestasRemovidas2 + custoAumentadoArestasIncluidas1 + custoAumentadoArestasIncluidas2;

		// atualiza custo das sub-rotas dos caminhões
		rotaAlterada[indCaminhao1][CVRPsolverConstantes::IND_DIST_PERCORRIDA] +=
				custoArestasIncluidas1 - custoArestasRemovidas1;
		rotaAlterada[indCaminhao2][CVRPsolverConstantes::IND_DIST_PERCORRIDA] +=
				custoArestasIncluidas2 - custoArestasRemovidas2;

	} else { // caso de troca entre vértices sucessivos

		// identifica vértices antecessores e sucessores
		int antecessorLocal1 = indLocal1 == 0 ? 0 :
				rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1 - 1];
		int sucessorLocal2 = indLocal2 == rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_NUM_LOCAIS] - 1 ?
				0 : rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2 + 1];

		// calcula custos das arestas removidas
		int custoArestasRemovidas = this->distancias[antecessorLocal1][local1] +
				this->distancias[local2][sucessorLocal2] + this->distancias[local1][local2];

		// calcula custos aumentados das arestas removidas
		double custoAumentadoArestasRemovidas = custoArestasRemovidas + this->lambda *
				(this->penalidades[antecessorLocal1][local1] + this->penalidades[local2][sucessorLocal2] +
				 this->penalidades[local1][local2]);

		// calcula custos das arestas incluídas
		int custoArestasIncluidas = this->distancias[antecessorLocal1][local2] +
				this->distancias[local1][sucessorLocal2] + this->distancias[local2][local1];

		// calcula custos aumentados das arestas incluídas
		double custoAumentadoArestasIncluidas = custoArestasIncluidas + this->lambda *
				(this->penalidades[antecessorLocal1][local2] + this->penalidades[local1][sucessorLocal2] +
				 this->penalidades[local2][local1]);

		// calcula custos da rota alterada
		*custoRotaAlterada = custoRotaOriginal - custoArestasRemovidas + custoArestasIncluidas;
		*custoAumentadoRotaAlterada = custoAumentadoRotaOriginal - custoAumentadoArestasRemovidas +
				custoAumentadoArestasIncluidas;

		// atualiza custo das sub-rotas dos caminhões
		rotaAlterada[indCaminhao1][CVRPsolverConstantes::IND_DIST_PERCORRIDA] +=
				custoArestasIncluidas - custoArestasRemovidas;

	}

	// teste para correção de erro
//	if (*custoAumentadoRotaAlterada != this->calcularCustoAumentadoSolucaoPorDistancias(rotaAlterada))
//		printf("Erro\n");

	// atualiza cargas dos caminhões
	if (indCaminhao1 != indCaminhao2) {
		rotaAlterada[indCaminhao1][CVRPsolverConstantes::IND_CARGA] +=
				this->demandas[local2] - this->demandas[local1];
		rotaAlterada[indCaminhao2][CVRPsolverConstantes::IND_CARGA] +=
				this->demandas[local1] - this->demandas[local2];
	}

	return true;
}

bool CVRPsolver::executarInsercao(int** rotaOriginal, int custoRotaOriginal, double custoAumentadoRotaOriginal,
		int** rotaAlterada, int* custoRotaAlterada, double* custoAumentadoRotaAlterada,
		int indCaminhao1, int indLocal1, int indCaminhao2, int indLocal2) {
	// identifica item a remover do caminhão 1
	int local1 = rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1];

	// verifica viabilidade da inserção
	if (rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_CARGA] + this->demandas[local1] > this->capacidade)
		return false;

	// procede criação de rota alterada...

	// copiar rota original para a que será alterada
	//this->copiarRota(rotaOriginal, rotaAlterada);
	for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
		int quantidadeLocais = rotaOriginal[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
		if (quantidadeLocais == 0) {
			rotaAlterada[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
			break;
		}

		memcpy(rotaAlterada[indCaminhao], rotaOriginal[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
		if (rotaAlterada[indCaminhao][0] != rotaOriginal[indCaminhao][0])
			std::printf("Erro.\n");
	}


	// identifica item antecessor ao item a remover do caminhão 1
	int antecessor1 = indLocal1 == 0 ? 0 :
			rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1 - 1];

	// identifica item sucessor ao item a remover do caminhão 1
	int sucessor1 = indLocal1 == rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_NUM_LOCAIS] - 1 ? 0 :
			rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1 + 1];

	// identifica item da posição da rota que receberá o local1 (será o novo sucessor do local1)
	int sucessor2 = rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2];

	// identifica item da rota que antecede a posição que receberá o local1 (será o novo antecessor do local1)
	int antecessor2 = indLocal2 == 0 ? 0 :
			rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2 - 1];

	// faz ajuste de sucessor2 e antecessor2 quando a inserção é no mesmo caminhão e em posição depois da atual
	if (indCaminhao1 == indCaminhao2 && indLocal1 < indLocal2) {
		sucessor2 = indLocal2 == rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_NUM_LOCAIS] - 1 ? 0 :
				rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2 + 1];
		antecessor2 = rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2];
	}

	// remove item do caminhão 1
	for (int i = indLocal1 + 1; i < rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_NUM_LOCAIS]; i++)
		rotaAlterada[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + i - 1] =
				rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + i];
	(rotaAlterada[indCaminhao1][CVRPsolverConstantes::IND_NUM_LOCAIS])--;

	// insere item no caminhão 2
	for (int i = rotaOriginal[indCaminhao2][CVRPsolverConstantes::IND_NUM_LOCAIS] - 1; i >= indLocal2; i--)
		rotaAlterada[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + i + 1] =
				rotaAlterada[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + i];
	rotaAlterada[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2] = local1;

	(rotaAlterada[indCaminhao2][CVRPsolverConstantes::IND_NUM_LOCAIS])++;

	// atualiza distâncias percorridas

	int custoArestasRemovidasLocalRemocao = this->distancias[antecessor1][local1] + this->distancias[local1][sucessor1];
	int custoArestaIncluidaLocalRemocao = this->distancias[antecessor1][sucessor1];
	double custoAumentadoArestasRemovidasLocalRemocao = custoArestasRemovidasLocalRemocao + this->lambda *
			(this->penalidades[antecessor1][local1] + this->penalidades[local1][sucessor1]);
	double custoAumentadoArestaIncluidaLocalRemocao = custoArestaIncluidaLocalRemocao + this->lambda *
			this->penalidades[antecessor1][sucessor1];

	rotaAlterada[indCaminhao1][CVRPsolverConstantes::IND_DIST_PERCORRIDA] += custoArestaIncluidaLocalRemocao -
			custoArestasRemovidasLocalRemocao;

	int custoArestaRemovidaLocalInsercao = this->distancias[antecessor2][sucessor2];
	int custoArestasIncluidasLocalInsercao = this->distancias[antecessor2][local1] +
			this->distancias[local1][sucessor2];
	double custoAumentadoArestaRemovidaLocalInsercao = custoArestaRemovidaLocalInsercao + this->lambda *
			this->penalidades[antecessor2][sucessor2];
	double custoAumentadoArestasIncluidasLocalInsercao = custoArestasIncluidasLocalInsercao + this->lambda *
			(this->penalidades[antecessor2][local1] + this->penalidades[local1][sucessor2]);

	rotaAlterada[indCaminhao2][CVRPsolverConstantes::IND_DIST_PERCORRIDA] += custoArestasIncluidasLocalInsercao -
			custoArestaRemovidaLocalInsercao;

	// calcula custo da rota alterada

	*custoRotaAlterada = custoRotaOriginal - custoArestasRemovidasLocalRemocao + custoArestaIncluidaLocalRemocao -
			custoArestaRemovidaLocalInsercao + custoArestasIncluidasLocalInsercao;
	*custoAumentadoRotaAlterada = custoAumentadoRotaOriginal - custoAumentadoArestasRemovidasLocalRemocao +
			custoAumentadoArestaIncluidaLocalRemocao - custoAumentadoArestaRemovidaLocalInsercao +
			custoAumentadoArestasIncluidasLocalInsercao;

	// atualiza cargas

	rotaAlterada[indCaminhao1][CVRPsolverConstantes::IND_CARGA] -= this->demandas[local1];
	rotaAlterada[indCaminhao2][CVRPsolverConstantes::IND_CARGA] += this->demandas[local1];

	// verifica se houve redução do número de caminhões
	if (indCaminhao1 != indCaminhao2 &&
			rotaOriginal[indCaminhao1][CVRPsolverConstantes::IND_NUM_LOCAIS] == 1) {
		// faz o deslocamento das rotas dos caminhões subsequentes para manter todas as rotas não nulas em posições contínuas
		for (int i = indCaminhao1; rotaOriginal[i][CVRPsolverConstantes::IND_NUM_LOCAIS] != 0; i++) {
			int quantidadeLocais = rotaAlterada[i + 1][CVRPsolverConstantes::IND_NUM_LOCAIS];
			memcpy(rotaAlterada[i], rotaAlterada[i + 1], (quantidadeLocais + 3) * sizeof(int));
		}
	}


	return true;
}

double arredondar(double num, int casasDecimais) {
	int b = 1;
	for (int i = 0; i < casasDecimais; i++)
		b = b * 10;
	return floor(num * b + 0.5) / b;
}

int** CVRPsolver::localSearch(int** s0, int** melhorSolucaoEmG, long int* quantSolucoesAvaliadas, int* quantMelhorias,
		bool* melhoriaEmG, bool permitirNovasRotas) {
	if (this->solucoesAuxiliaresNulas) {
		this->vizinho = this->gerarSolucaoVazia();
		this->melhorVizinho = this->gerarSolucaoVazia();
		this->otimoLocal = this->gerarSolucaoVazia();
		this->solucoesAuxiliaresNulas = false;
	}

	this->copiarRota(s0, this->otimoLocal);

	int custoVizinho = std::numeric_limits<int>::max();
	int custoMelhorVizinho = std::numeric_limits<int>::max();
	int custoOtimoLocal = this->calcularCustoSolucao(s0, false, false);
	int custoMelhorSolucaoEmG = this->calcularCustoSolucao(melhorSolucaoEmG, false, false);

	double custoAumentadoVizinho = std::numeric_limits<double>::max();
	double custoAumentadoMelhorVizinho = std::numeric_limits<double>::max();
	double custoAumentadoOtimoLocal = this->calcularCustoSolucao(s0, true, false);

//	*quantMelhorias = -1;
//	*quantSolucoesAvaliadas = 0;

	bool houveMelhoria;
	do {
		(*quantMelhorias)++;
		houveMelhoria = false;

		// obtém número de caminhões de otimoLocal
		int numCaminhoes = 0;
		for (int i = 0; i < this->numLocais; i++)
			if (this->otimoLocal[i][CVRPsolverConstantes::IND_NUM_LOCAIS] > 0)
				numCaminhoes++;
			else
				break;

		// para o caso de estar executando busca local rápida e com melhor aprimorante:
		int localTrocado1 = -1;
		int localTrocado2 = -1;

		// ----------------------------------------------------------------------- procura na vizinhança "2opt"

		for (int indCaminhao = 0; indCaminhao < numCaminhoes; indCaminhao++) {
			int numLocaisCaminhao = this->otimoLocal[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
			for (int indLocal1 = 0; indLocal1 < numLocaisCaminhao - 1; indLocal1++) {
				int local = otimoLocal[indCaminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1];

				// para caso de busca local rápida:
				if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
					if (!vizinhancaAtivada[local])
						continue;
					vizinhancaTemSolucaoMelhor[local] = false;
				}

				for (int indLocal2 = indLocal1 + 1; indLocal2 < numLocaisCaminhao; indLocal2++) {
					int localTrocar = otimoLocal[indCaminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2];
					executar2Opt(this->otimoLocal, custoOtimoLocal, custoAumentadoOtimoLocal, this->vizinho, &custoVizinho,
							&custoAumentadoVizinho, indCaminhao, indLocal1, indLocal2);
					(*quantSolucoesAvaliadas)++;

					// verifica melhoria na função de custo G
					if (custoVizinho < custoMelhorSolucaoEmG) {
//						this->copiarRota(this->vizinho, melhorSolucaoEmG);
						for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
							int quantidadeLocais = vizinho[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
							if (quantidadeLocais == 0) {
								melhorSolucaoEmG[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
								break;
							}

							memcpy(melhorSolucaoEmG[indCaminhao], vizinho[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
							if (melhorSolucaoEmG[indCaminhao][0] != vizinho[indCaminhao][0])
								std::printf("Erro.\n");
						}

						custoMelhorSolucaoEmG = custoVizinho;
						*melhoriaEmG = true;
					}

					// verifica melhoria em ótimo local
					switch (this->opcaoAprimorante) {
					case CVRPsolverConstantes::OPCAO_MELHOR_APRIMORANTE:
						if (arredondar(custoAumentadoVizinho, 4) < arredondar(custoAumentadoMelhorVizinho, 4)) {
//							this->copiarRota(this->vizinho, this->melhorVizinho);
							for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
								int quantidadeLocais = vizinho[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
								if (quantidadeLocais == 0) {
									melhorVizinho[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
									break;
								}

								memcpy(melhorVizinho[indCaminhao], vizinho[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
								if (melhorVizinho[indCaminhao][0] != vizinho[indCaminhao][0])
									std::printf("Erro.\n");
							}

							custoMelhorVizinho = custoVizinho;
							custoAumentadoMelhorVizinho = custoAumentadoVizinho;
							if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
								vizinhancaTemSolucaoMelhor[local] = true;
								localTrocado1 = local;
								localTrocado2 = localTrocar;
							}
						}
						break;
					case CVRPsolverConstantes::OPCAO_PRIMEIRO_APRIMORANTE:
						if (arredondar(custoAumentadoVizinho, 4) < arredondar(custoAumentadoOtimoLocal, 4)) {
//							this->copiarRota(this->vizinho, this->otimoLocal);
							for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
								int quantidadeLocais = vizinho[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
								if (quantidadeLocais == 0) {
									otimoLocal[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
									break;
								}

								memcpy(otimoLocal[indCaminhao], vizinho[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
								if (otimoLocal[indCaminhao][0] != vizinho[indCaminhao][0])
									std::printf("Erro.\n");
							}

							custoOtimoLocal = custoVizinho;
							custoAumentadoOtimoLocal = custoAumentadoVizinho;
							houveMelhoria = true;
							if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
								vizinhancaTemSolucaoMelhor[local] = true;
								vizinhancaAtivada[local] = true;
								vizinhancaAtivada[localTrocar] = true;
							}
						}
						break;
					}

					if (houveMelhoria)
						break;
				}

				if (houveMelhoria)
					break;
			}

			if (houveMelhoria)
				break;
		}

		if (houveMelhoria) // verificação necessária para opção de primeiro aprimorante
			continue; // passa para próxima iteração do do...while

		// ----------------------------------------------------------------------- procura na vizinhança "troca"

		for (int indCaminhao1 = 0; indCaminhao1 < numCaminhoes; indCaminhao1++) {
			int numLocaisCaminhao1 = this->otimoLocal[indCaminhao1][CVRPsolverConstantes::IND_NUM_LOCAIS];
			for (int indLocal1 = 0; indLocal1 < numLocaisCaminhao1; indLocal1++) {
				int local1 = otimoLocal[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1];

				// para caso de busca local rápida:
				if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
					if (!vizinhancaAtivada[local1])
						continue;
				}

				for (int indCaminhao2 = indCaminhao1; indCaminhao2 < numCaminhoes; indCaminhao2++) {
					int numLocaisCaminhao2 = this->otimoLocal[indCaminhao2][CVRPsolverConstantes::IND_NUM_LOCAIS];
					int inicioLocal2 = (indCaminhao2 == indCaminhao1) ? indLocal1 + 1 : 0;
					for (int indLocal2 = inicioLocal2; indLocal2 < numLocaisCaminhao2; indLocal2++) {
						int localTrocar = otimoLocal[indCaminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal2];
						if (this->executarTroca(this->otimoLocal, custoOtimoLocal, custoAumentadoOtimoLocal,
								this->vizinho, &custoVizinho, &custoAumentadoVizinho, indCaminhao1, indLocal1,
								indCaminhao2, indLocal2)) {
							(*quantSolucoesAvaliadas)++;

							// verifica melhoria na função de custo G
							if (custoVizinho < custoMelhorSolucaoEmG) {
		//						this->copiarRota(this->vizinho, melhorSolucaoEmG);
								for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
									int quantidadeLocais = vizinho[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
									if (quantidadeLocais == 0) {
										melhorSolucaoEmG[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
										break;
									}

									memcpy(melhorSolucaoEmG[indCaminhao], vizinho[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
									if (melhorSolucaoEmG[indCaminhao][0] != vizinho[indCaminhao][0])
										std::printf("Erro.\n");
								}

								custoMelhorSolucaoEmG = custoVizinho;
								*melhoriaEmG = true;
							}

							// verifica melhoria em ótimo local
							switch (this->opcaoAprimorante) {
							case CVRPsolverConstantes::OPCAO_MELHOR_APRIMORANTE:
								if (arredondar(custoAumentadoVizinho, 4) < arredondar(custoAumentadoMelhorVizinho, 4)) {
		//							this->copiarRota(this->vizinho, this->melhorVizinho);
									for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
										int quantidadeLocais = vizinho[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
										if (quantidadeLocais == 0) {
											melhorVizinho[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
											break;
										}

										memcpy(melhorVizinho[indCaminhao], vizinho[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
										if (melhorVizinho[indCaminhao][0] != vizinho[indCaminhao][0])
											std::printf("Erro.\n");
									}

									custoMelhorVizinho = custoVizinho;
									custoAumentadoMelhorVizinho = custoAumentadoVizinho;
									if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
										vizinhancaTemSolucaoMelhor[local1] = true;
										localTrocado1 = local1;
										localTrocado2 = localTrocar;
									}
								}
								break;
							case CVRPsolverConstantes::OPCAO_PRIMEIRO_APRIMORANTE:
								if (arredondar(custoAumentadoVizinho, 4) < arredondar(custoAumentadoOtimoLocal, 4)) {
		//							this->copiarRota(this->vizinho, this->otimoLocal);
									for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
										int quantidadeLocais = vizinho[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
										if (quantidadeLocais == 0) {
											otimoLocal[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
											break;
										}

										memcpy(otimoLocal[indCaminhao], vizinho[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
										if (otimoLocal[indCaminhao][0] != vizinho[indCaminhao][0])
											std::printf("Erro.\n");
									}

									custoOtimoLocal = custoVizinho;
									custoAumentadoOtimoLocal = custoAumentadoVizinho;
									houveMelhoria = true;
									if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
										vizinhancaTemSolucaoMelhor[local1] = true;
										vizinhancaAtivada[local1] = true;
										vizinhancaAtivada[localTrocar] = true;
									}
								}
								break;
							}

							if (houveMelhoria)
								break;
						}
					}

					if (houveMelhoria)
						break;
				}

				if (houveMelhoria)
					break;
			}

			if (houveMelhoria)
				break;
		}

		if (houveMelhoria) // verificação necessária para opção de primeiro aprimorante
			continue; // passa para próxima iteração do do...while

		// ----------------------------------------------------------------------- procura na vizinhança "inserção"

		for (int indCaminhao1 = 0; indCaminhao1 < numCaminhoes; indCaminhao1++) {
			int numLocaisCaminhao1 = this->otimoLocal[indCaminhao1][CVRPsolverConstantes::IND_NUM_LOCAIS];
			for (int indLocal1 = 0; indLocal1 < numLocaisCaminhao1; indLocal1++) {
				int local1 = otimoLocal[indCaminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1];

				// para caso de busca local rápida:
				if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
					if (!vizinhancaAtivada[local1])
						continue;
				}

				int limiteRotas = numCaminhoes;
				if (permitirNovasRotas) {
					limiteRotas++;
				}
				for (int indCaminhao2 = 0; indCaminhao2 < limiteRotas; indCaminhao2++) {
					int numLocaisCaminhao2 = this->otimoLocal[indCaminhao2][CVRPsolverConstantes::IND_NUM_LOCAIS];
					for (int indLocal2 = 0; indLocal2 < numLocaisCaminhao2; indLocal2++) {
						// descarta caso de inserir na mesma posição
						if (indCaminhao1 == indCaminhao2 && indLocal1 == indLocal2)
							continue;

						// descarta caso de inserir em posições adjacentes (caso tratado por troca)
						if (indCaminhao1 == indCaminhao2 && abs(indLocal1 - indLocal2) == 1)
							continue;

						// descarta caso de redução de número de caminhões abaixo do mínimo
						if (numCaminhoes == this->numMinDeCaminhoes && indCaminhao1 != indCaminhao2 &&
								this->otimoLocal[indCaminhao1][CVRPsolverConstantes::IND_NUM_LOCAIS] == 1)
							continue;

						// incluir condição para priorizar vizinhos com menor número de caminhões <------------------ !!!
						if (this->executarInsercao(this->otimoLocal, custoOtimoLocal, custoAumentadoOtimoLocal,
								this->vizinho, &custoVizinho, &custoAumentadoVizinho, indCaminhao1, indLocal1,
								indCaminhao2, indLocal2)) {

							(*quantSolucoesAvaliadas)++;

							// verifica melhoria na função de custo G
							if (custoVizinho < custoMelhorSolucaoEmG) {
//								this->copiarRota(this->vizinho, melhorSolucaoEmG);
								for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
									int quantidadeLocais = vizinho[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
									if (quantidadeLocais == 0) {
										melhorSolucaoEmG[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
										break;
									}

									memcpy(melhorSolucaoEmG[indCaminhao], vizinho[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
									if (melhorSolucaoEmG[indCaminhao][0] != vizinho[indCaminhao][0])
										std::printf("Erro.\n");
								}
								custoMelhorSolucaoEmG = custoVizinho;
								*melhoriaEmG = true;
							}

							// verifica melhoria em ótimo local
							switch (this->opcaoAprimorante) {
							case CVRPsolverConstantes::OPCAO_MELHOR_APRIMORANTE:
								if (arredondar(custoAumentadoVizinho, 4) < arredondar(custoAumentadoMelhorVizinho, 4)) {
		//							this->copiarRota(this->vizinho, this->melhorVizinho);
									for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
										int quantidadeLocais = vizinho[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
										if (quantidadeLocais == 0) {
											melhorVizinho[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
											break;
										}

										memcpy(melhorVizinho[indCaminhao], vizinho[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
										if (melhorVizinho[indCaminhao][0] != vizinho[indCaminhao][0])
											std::printf("Erro.\n");
									}

									custoMelhorVizinho = custoVizinho;
									custoAumentadoMelhorVizinho = custoAumentadoVizinho;
									if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
										vizinhancaTemSolucaoMelhor[local1] = true;
									}
								}
								break;
							case CVRPsolverConstantes::OPCAO_PRIMEIRO_APRIMORANTE:
								if (arredondar(custoAumentadoVizinho, 4) < arredondar(custoAumentadoOtimoLocal, 4)) {
		//							this->copiarRota(this->vizinho, this->otimoLocal);
									for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
										int quantidadeLocais = vizinho[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
										if (quantidadeLocais == 0) {
											otimoLocal[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
											break;
										}

										memcpy(otimoLocal[indCaminhao], vizinho[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
										if (otimoLocal[indCaminhao][0] != vizinho[indCaminhao][0])
											std::printf("Erro.\n");
									}

									custoOtimoLocal = custoVizinho;
									custoAumentadoOtimoLocal = custoAumentadoVizinho;
									houveMelhoria = true;
									if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
										vizinhancaTemSolucaoMelhor[local1] = true;
									}
								}
								break;
							}
						}

						if (houveMelhoria)
							break;
					}
					// para o caso de nova rota (não terá feito iteração do "for" acima
					if (numLocaisCaminhao2 == 0 && otimoLocal[indCaminhao1][CVRPsolverConstantes::IND_NUM_LOCAIS] > 1) {
						this->executarInsercaoEmNovaRota(this->otimoLocal, custoOtimoLocal, custoAumentadoOtimoLocal,
								this->vizinho, &custoVizinho, &custoAumentadoVizinho, indCaminhao1, indLocal1,
								indCaminhao2);

						(*quantSolucoesAvaliadas)++;

						// verifica melhoria na função de custo G
						if (custoVizinho < custoMelhorSolucaoEmG) {
							this->copiarRota(this->vizinho, melhorSolucaoEmG);
							custoMelhorSolucaoEmG = custoVizinho;
							*melhoriaEmG = true;
						}

						// verifica melhoria em ótimo local
						switch (this->opcaoAprimorante) {
						case CVRPsolverConstantes::OPCAO_MELHOR_APRIMORANTE:
							if (arredondar(custoAumentadoVizinho, 4) < arredondar(custoAumentadoMelhorVizinho, 4)) {
								this->copiarRota(this->vizinho, this->melhorVizinho);
								custoMelhorVizinho = custoVizinho;
								custoAumentadoMelhorVizinho = custoAumentadoVizinho;
								if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
									vizinhancaTemSolucaoMelhor[local1] = true;
								}
							}
							break;
						case CVRPsolverConstantes::OPCAO_PRIMEIRO_APRIMORANTE:
							if (arredondar(custoAumentadoVizinho, 4) < arredondar(custoAumentadoOtimoLocal, 4)) {
								this->copiarRota(this->vizinho, this->otimoLocal);
								custoOtimoLocal = custoVizinho;
								custoAumentadoOtimoLocal = custoAumentadoVizinho;
								houveMelhoria = true;
								if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
									vizinhancaTemSolucaoMelhor[local1] = true;
								}
							}
							break;
						}
					}

					if (houveMelhoria)
						break;
				}

				// para o caso de busca local rápida:
				if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
					if (!vizinhancaTemSolucaoMelhor[local1])
						vizinhancaAtivada[local1] = false;
				}

				if (houveMelhoria)
					break;
			}

			if (houveMelhoria)
				break;
		}


		if (houveMelhoria) // verificação necessária para opção de primeiro aprimorante
			continue; // passa para próxima iteração do do...while

		// ----------------------------------------------------------------------- fim de procura em vizinhanças

		// verifica melhoria (necessário para opção de melhor aprimorante)
		if (this->opcaoAprimorante == CVRPsolverConstantes::OPCAO_MELHOR_APRIMORANTE &&
				custoAumentadoMelhorVizinho < custoAumentadoOtimoLocal) {
			this->copiarRota(this->melhorVizinho, this->otimoLocal);
			custoOtimoLocal = custoMelhorVizinho;
			custoAumentadoOtimoLocal = custoAumentadoMelhorVizinho;
			houveMelhoria = true;
		}

	} while (houveMelhoria);

	return this->otimoLocal;
}

double CVRPsolver::calcularCustoSolucao(int** solucao, bool usarPenalidades, bool aplicarCustoCaminhoes) {
	int numCaminhoes = 0;
	double custo = 0;

	// calcular custo
	for (int caminhao = 0; caminhao < this->numLocais; caminhao++) {
		if (solucao[caminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] == 0)
			break;
		numCaminhoes++;
		int custoCaminhao = solucao[caminhao][CVRPsolverConstantes::IND_DIST_PERCORRIDA];
		custo += custoCaminhao;
	}

	// calcular termo de regularização (penalidades)
	if (usarPenalidades) {
		double penalidade = 0;
		for (int caminhao = 0; caminhao < this->numLocais; caminhao++) {
			int ultimoLocal = this->indDeposito; // partida do depósito
			for (int local = 0; local < solucao[caminhao][CVRPsolverConstantes::IND_NUM_LOCAIS]; local++) {
				int localAtual = solucao[caminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + local];
				penalidade += this->penalidades[ultimoLocal][localAtual];
				ultimoLocal = localAtual; // atualiza último local
			}
			penalidade += this->penalidades[ultimoLocal][this->indDeposito]; // volta ao depósito
		}
		custo += this->lambda * penalidade; // termo de regularização
	}

	// aplicar custo de número de caminhões
	if (aplicarCustoCaminhoes)
		custo += (numCaminhoes - 1) * this->distanciaMaxima * this->numLocais;

	return custo;
}

double CVRPsolver::expressaoUtilidade(int origem, int destino) {
	double util = ((double)this->distancias[origem][destino]) / (1 + this->penalidades[origem][destino]);
	return util;
}

void gotoxy(int x,int y) {
	printf("%c[%d;%df",0x1B,y,x);
}

double CVRPsolver::get_cpu_time(){
    return (double)clock() / CLOCKS_PER_SEC;
}

int** CVRPsolver::guidedLocalSearch(std::string nomeMetodoBuscaLocal, std::string opcaoAprimoramento) {
	// para caso de salvar dados de progresso da execução
	int indIteracaoAgrupamentoProgressoExecucao = 0;
	int indAgrupamentoProgressoExecucao = 0;
	numAgrupamentosProgressoExecucao = 100;
	int numIteracoesAgrupamentoProgresso = (int)(numMaxIteracoes / numAgrupamentosProgressoExecucao);
	if (salvarDadosDeProgressoDaExecucao) {
		custoMelhorSolucao = new int[numAgrupamentosProgressoExecucao];
		for (int i = 0; i < numAgrupamentosProgressoExecucao; i++) {
			custoMelhorSolucao[i] = 0;
		}
	}

	if (opcaoAprimoramento == "P") {
		this->opcaoAprimorante = CVRPsolverConstantes::OPCAO_PRIMEIRO_APRIMORANTE;
	} else if (opcaoAprimoramento == "M") {
		this->opcaoAprimorante = CVRPsolverConstantes::OPCAO_MELHOR_APRIMORANTE;
	}
	if (nomeMetodoBuscaLocal == "BLC") {
		this->opcaoBuscaLocal = CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_CONVENCIONAL;
	} else if (nomeMetodoBuscaLocal == "FLS") {
		this->opcaoBuscaLocal = CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA;
		iniciarVetorDeAtivacaoDeVizinhanca();
	}

//	int** solucaoAtual = this->gerarSolucaoAleatoria();
	int** solucaoAtual = this->gerarSolucaoGulosa();
	int** melhorSolucaoEmG = this->gerarSolucaoVazia();
	this->copiarRota(solucaoAtual, melhorSolucaoEmG);

	this->iniciarPenalidades();
	double** util = new double*[this->numLocais];
	for (int i = 0; i < this->numLocais; i++)
		util[i] = new double[this->numLocais];

	if (this->imprimirAcompanhamento) {
		// limpar tela
		printf("%s", std::string(100, '\n').c_str());
		// imprimir cabeçalho
		gotoxy(0, 0);
		//      0            13                      37                                71                     94             109                  130
		printf("+---------------------+----------------------+-------------------------------+---------------------+-----------------+----------------------+\n");
		printf("| Iteração:  xxxxxxxx | Avaliações:  xxxxxxx | Iterações busca local:  xxxxx | Penalizações:  xxxx | Tempo:  xxxxxxx | Instância: x-xxxx-xx |\n");
		printf("+---------------------+----------------------+-------------------------------+---------------------+-----------------+----------------------+\n");
		printf("\n");
		printf("--------------[ Melhor solução ]--------------\n");
		//                                           37
		printf("                              Custo: xxxxxx   \n");
		printf("                           Iteração: xxxxxxxx \n");
		printf(" > Rotas: xx                                  \n");
		for (int i = 0; i < ((int)(numMinDeCaminhoes * 1.3 + 0.5)); i++)
			printf("\n");
		printf("\n");
		printf("--------------[ Ótimo local    ]--------------\n");
		printf("                              Custo: xxxxxx   \n");
		printf("                    Custo aumentado: xxxxxx   \n");
		printf(" > Rotas: xx                                  \n");
		for (int i = 0; i < ((int)(numMinDeCaminhoes * 1.3 + 0.5)); i++)
			printf("\n");

		gotoxy(131, 2);
		printf("%9s", nomeInstancia.c_str());
	}

	long int contIteracoes = 1;
	int contIteracoesSemMelhoria = 0;
	int iteracaoUltimaMelhoriaEmG = 0;
	quantidadeSolucoesAvaliadas = 0;
	int quantidadeMelhorias = 0;
	int quantSolAvalUltimaIteracao = 0;
	int quantMelhoriasUltimaIteracao = 0;
	while (contIteracoes <= this->numMaxIteracoes) {

		if (this->imprimirAcompanhamento) {
			gotoxy(14, 2);
			std::printf("%8d", contIteracoes);
		}

		// reinicialização de penalidades (se configurado para tal)
		if (iteracoesAteReiniciarPenalidades > 0 && contIteracoes % iteracoesAteReiniciarPenalidades == 0)
			zerarPenalidades();

		bool melhoriaEmG = false;

		// faz busca local
		int** solucaoSeguinte = this->localSearch(solucaoAtual, melhorSolucaoEmG,
				&quantidadeSolucoesAvaliadas, &quantidadeMelhorias, &melhoriaEmG, true);
		this->copiarRota(solucaoSeguinte, solucaoAtual);

		// para o caso de salvar dados do progresso da execução
		if (salvarDadosDeProgressoDaExecucao) {
			indIteracaoAgrupamentoProgressoExecucao++;
			if (indIteracaoAgrupamentoProgressoExecucao == numIteracoesAgrupamentoProgresso) {
				custoMelhorSolucao[indAgrupamentoProgressoExecucao] = calcularCustoSolucao(melhorSolucaoEmG, false, false);
				indIteracaoAgrupamentoProgressoExecucao = 0;
				indAgrupamentoProgressoExecucao++;
			}
		}

		// conta iterações sem melhoria na função G
		if (!melhoriaEmG)
			contIteracoesSemMelhoria++;
		else
			iteracaoUltimaMelhoriaEmG = contIteracoes;

		// se iterações sem melhoria atingir quantidade limite, ativar todas as vizinhanças
		if (iteracoesAteReiniciarPenalidades > 0 &&
				contIteracoesSemMelhoria >= iteracoesAteReiniciarPenalidades * 0.3) {
			ativarTodasAsVizinhancas();
			contIteracoesSemMelhoria = 0;
		}

		// processar penalizações...

		double maxUtil = 0;
		int numCaminhoes = 0;
		for (int i = 0; i < this->numLocais; i++)
			if (solucaoAtual[i][CVRPsolverConstantes::IND_NUM_LOCAIS] > 0)
				numCaminhoes++;
			else
				break;

		// calcular função de utilidade para cada aresta do ótimo local (solução atual)
		for (int indCaminhao = 0; indCaminhao < numCaminhoes; indCaminhao++) {
			int numLocaisCaminhao = solucaoAtual[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
			int antecessor = 0;
			int atual = 0;
			for (int indLocal = 0; indLocal <= numLocaisCaminhao; indLocal++) {
				if (indLocal == numLocaisCaminhao)
					atual = 0;
				else
					atual = solucaoAtual[indCaminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal];

				util[antecessor][atual] = this->expressaoUtilidade(antecessor, atual);
				if (util[antecessor][atual] > maxUtil)
					maxUtil = util[antecessor][atual];

				antecessor = atual;
			}
		}

		// aplicar penalizações às arestas que maximizam a função de utilidade
		int contPenalizacoes = 0;
		for (int indCaminhao = 0; indCaminhao < numCaminhoes; indCaminhao++) {
			int numLocaisCaminhao = solucaoAtual[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
			int antecessor = 0;
			int atual = 0;
			for (int indLocal = 0; indLocal <= numLocaisCaminhao; indLocal++) {
				if (indLocal == numLocaisCaminhao)
					atual = 0;
				else
					atual = solucaoAtual[indCaminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal];

				if (arredondar(util[antecessor][atual], 4) == arredondar(maxUtil, 4)) {
					(this->penalidades[antecessor][atual])++;
					if (penalidadesSimetricas)
						(this->penalidades[atual][antecessor])++;
					contPenalizacoes++;

					// para o caso de busca local rápida:
					if (opcaoBuscaLocal == CVRPsolverConstantes::OPCAO_BUSCA_LOCAL_RAPIDA) {
						vizinhancaAtivada[antecessor] = true;
						vizinhancaAtivada[atual] = true;
					}
				}

				antecessor = atual;
			}
		}

		if (this->imprimirAcompanhamento) {
//			std::printf("- penaliz.: %3d - aval.: %7d - iter.b.l.: %4d - custo: %6d",
//					contPenalizacoes, quantidadeSolucoesAvaliadas - quantSolAvalUltimaIteracao,
//					quantidadeMelhorias - quantMelhoriasUltimaIteracao,
//					(int)this->calcularCustoSolucao(melhorSolucaoEmG, false, false));
			gotoxy(37, 2);
			std::printf("%8d", quantidadeSolucoesAvaliadas - quantSolAvalUltimaIteracao);
			gotoxy(72, 2);
			std::printf("%5d", quantidadeMelhorias - quantMelhoriasUltimaIteracao);
			gotoxy(95, 2);
			std::printf("%4d", contPenalizacoes);
			gotoxy(110, 2);
			std::printf("%7.2f", get_cpu_time());

			if (melhoriaEmG) {
				gotoxy(38, 6);
				std::printf("%-6d", (int)calcularCustoSolucao(melhorSolucaoEmG, false, false));
				gotoxy(38, 7);
				std::printf("%-8d", iteracaoUltimaMelhoriaEmG);
				gotoxy(11, 8);
				int numVeiculos = obterNumCaminhoesSolucao(melhorSolucaoEmG);
				std::printf("%-2d", numVeiculos);
				for (int r = 0; r < numVeiculos; r++) {
					gotoxy(0, 9 + r);
					std::printf(" > Rota #%2d [%4d]: ", r + 1, melhorSolucaoEmG[r][CVRPsolverConstantes::IND_CARGA]);
					gotoxy(21, 9 + r);
					int numCaracteres = 21;
					int numLocaisRota = melhorSolucaoEmG[r][CVRPsolverConstantes::IND_NUM_LOCAIS];
					for (int l = 0; l < numLocaisRota; l++) {
						std::printf("%s%d", (l > 0 ? " " : ""),
								melhorSolucaoEmG[r][CVRPsolverConstantes::IND_INICIO_LOCAIS + l]);
						numCaracteres += l > 0 ? 1 : 0;
						numCaracteres += (int)log10(melhorSolucaoEmG[r][CVRPsolverConstantes::IND_INICIO_LOCAIS + l]);
					}
					if (numCaracteres < 150)
						printf("%s",std::string(150 - numCaracteres, ' ').c_str());
				}
			}

			int linhaInicioOtimoLocal = 11 + ((int)(numMinDeCaminhoes * 1.3 + 0.5));
			gotoxy(38, linhaInicioOtimoLocal);
			std::printf("%-6d", (int)calcularCustoSolucao(solucaoSeguinte, false, false));
			gotoxy(38, linhaInicioOtimoLocal + 1);
			std::printf("%-6d", (int)calcularCustoSolucao(solucaoSeguinte, true, false));
			gotoxy(11, linhaInicioOtimoLocal + 2);
			int numVeiculos = obterNumCaminhoesSolucao(solucaoSeguinte);
			std::printf("%-2d", numVeiculos);
			for (int r = 0; r < numVeiculos; r++) {
				gotoxy(0, linhaInicioOtimoLocal + 3 + r);
				std::printf(" > Rota #%2d [%4d]: ", r + 1, solucaoSeguinte[r][CVRPsolverConstantes::IND_CARGA]);
				gotoxy(21, linhaInicioOtimoLocal + 3 + r);
				int numCaracteres = 21;
				int numLocaisRota = solucaoSeguinte[r][CVRPsolverConstantes::IND_NUM_LOCAIS];
				for (int l = 0; l < numLocaisRota; l++) {
					std::printf("%s%d", (l > 0 ? " " : ""),
							solucaoSeguinte[r][CVRPsolverConstantes::IND_INICIO_LOCAIS + l]);
					numCaracteres += l > 0 ? 1 : 0;
					numCaracteres += (int)log10(solucaoSeguinte[r][CVRPsolverConstantes::IND_INICIO_LOCAIS + l]);
				}
				if (numCaracteres < 150)
					printf("%s",std::string(150 - numCaracteres, ' ').c_str());
			}
			gotoxy(1, linhaInicioOtimoLocal + 3 + numVeiculos);

			fflush(stdout);
		}

		// contagens para acompanhamento
		quantSolAvalUltimaIteracao = quantidadeSolucoesAvaliadas;
		quantMelhoriasUltimaIteracao = quantidadeMelhorias;

		contIteracoes++;
	}

	delete [] solucaoAtual;
	delete [] util;
	delete [] vizinhancaAtivada;
	delete [] penalidades;
	delete [] vizinho;
	delete [] melhorVizinho;
	delete [] otimoLocal;
	solucoesAuxiliaresNulas = true;

	printf("Iteração última melhoria: %d\n\n", iteracaoUltimaMelhoriaEmG);

	if (this->imprimirAcompanhamento)
		printf("\n -> Soluções avaliadas: %ld\n -> Iterações de busca local: %d\n\n",
				quantidadeSolucoesAvaliadas, quantidadeMelhorias);

	if (salvarDadosDeProgressoDaExecucao) {
		int numIteracoesNaoAcumuladas = numMaxIteracoes % numIteracoesAgrupamentoProgresso;
		if (numIteracoesNaoAcumuladas > 0)
			custoMelhorSolucao[indAgrupamentoProgressoExecucao] = calcularCustoSolucao(melhorSolucaoEmG, false, false);
	}

	return melhorSolucaoEmG;
}

void CVRPsolver::copiarRota(int** rotaOriginal, int** rotaCopia) {
	for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
		int quantidadeLocais = rotaOriginal[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
		if (quantidadeLocais == 0) {
			rotaCopia[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
			break;
		}

		memcpy(rotaCopia[indCaminhao], rotaOriginal[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
		if (rotaCopia[indCaminhao][0] != rotaOriginal[indCaminhao][0])
			std::printf("Erro.\n");
	}
}

int CVRPsolver::calcularCustoSolucaoPorDistancias(int** solucao) {
	int custo = 0;
	for (int c = 0; c < this->numLocais; c++) {
		int quantidadeLocais = solucao[c][CVRPsolverConstantes::IND_NUM_LOCAIS];
		if (quantidadeLocais == 0)
			break;
		int primeiroLocal = solucao[c][CVRPsolverConstantes::IND_INICIO_LOCAIS];
		int ultimoLocal = solucao[c][CVRPsolverConstantes::IND_INICIO_LOCAIS + quantidadeLocais - 1];

		custo += this->distancias[0][primeiroLocal];
		for (int l = 0; l < quantidadeLocais - 1; l++) {
			int local = solucao[c][CVRPsolverConstantes::IND_INICIO_LOCAIS + l];
			int proximoLocal = solucao[c][CVRPsolverConstantes::IND_INICIO_LOCAIS + l + 1];
			custo += this->distancias[local][proximoLocal];
		}
		custo += this->distancias[ultimoLocal][0];
	}
	return custo;
}

double CVRPsolver::calcularCustoAumentadoSolucaoPorDistancias(int** solucao) {
	double custo = 0;
	int custoG = this->calcularCustoSolucaoPorDistancias(solucao);
	for (int c = 0; c < this->numLocais; c++) {
		int quantidadeLocais = solucao[c][CVRPsolverConstantes::IND_NUM_LOCAIS];
		if (quantidadeLocais == 0)
			break;
		int primeiroLocal = solucao[c][CVRPsolverConstantes::IND_INICIO_LOCAIS];
		int ultimoLocal = solucao[c][CVRPsolverConstantes::IND_INICIO_LOCAIS + quantidadeLocais - 1];

		custo += this->penalidades[0][primeiroLocal];
		for (int l = 0; l < quantidadeLocais - 1; l++) {
			int local = solucao[c][CVRPsolverConstantes::IND_INICIO_LOCAIS + l];
			int proximoLocal = solucao[c][CVRPsolverConstantes::IND_INICIO_LOCAIS + l + 1];
			custo += this->penalidades[local][proximoLocal];
		}
		custo += this->penalidades[ultimoLocal][0];
	}
	custo = custoG + this->lambda * custo;
	return custo;
}

int CVRPsolver::calcularCustoSolucaoPorCaminhao(int** solucao, int indCaminhao) {
	int custo = 0;
	int numLocaisCaminhao = solucao[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
	int antecessor = 0;
	int atual = 0;
	for (int l = 0; l < numLocaisCaminhao + 1; l++) {
		if (l < numLocaisCaminhao)
			atual = solucao[indCaminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + l];
		else
			atual = 0;
		custo += this->distancias[antecessor][atual];
		antecessor = atual;
	}
	return custo;
}

double CVRPsolver::calcularTermoRegularizacaoSolucaoPorCaminhao(int** solucao, int indCaminhao) {
	int soma = 0;
	int numLocaisCaminhao = solucao[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
	int antecessor = 0;
	int atual = 0;
	for (int l = 0; l < numLocaisCaminhao + 1; l++) {
		if (l < numLocaisCaminhao)
			atual = solucao[indCaminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + l];
		else
			atual = 0;
		soma += this->penalidades[antecessor][atual];
		antecessor = atual;
	}
	double termo = this->lambda * soma;
	return termo;
}

int CVRPsolver::obterDistancia(int local1, int local2) {
	int menor = local1 <= local2 ? local1 : local2;
	int maior = local1 > local2 ? local1 : local2;
	int distancia = this->distancias[menor][maior];
	return distancia;
}

void CVRPsolver::iniciarPenalidades() {
	this->penalidades = new int*[this->numLocais];
	for (int i = 0; i < this->numLocais; i++) {
		this->penalidades[i] = new int[this->numLocais];
		for (int j = 0; j < this->numLocais; j++)
			this->penalidades[i][j] = 0;
	}
}

void CVRPsolver::setLambda(double lambda) {
	this->lambda = lambda;
}

int CVRPsolver::getNumLocais() {
	return this->numLocais;
}

int CVRPsolver::obterQuantidadeSolucoesAvaliadas() {
	return this->quantidadeSolucoesAvaliadas;
}

double CVRPsolver::getLambda() {
	return this->lambda;
}

void CVRPsolver::iniciarVetorDeAtivacaoDeVizinhanca() {
	vizinhancaAtivada = new bool[numLocais];
	vizinhancaTemSolucaoMelhor = new bool[numLocais];
	for (int i = 0; i < numLocais; i++) {
		vizinhancaAtivada[i] = true;
		vizinhancaTemSolucaoMelhor[i] = false;
	}
}

void CVRPsolver::zerarPenalidades() {
	for (int i = 0; i < this->numLocais; i++) {
		for (int j = 0; j < this->numLocais; j++)
			this->penalidades[i][j] = 0;
	}
}

void CVRPsolver::ativarTodasAsVizinhancas() {
	for (int i = 0; i < numLocais; i++)
		vizinhancaAtivada[i] = true;
}

/**
 * Gera uma solução seguindo o seguinte método heurístico:
 * 1º) Com base no número mínimo de veículos, atribui dois locais visitados (inicial e final) a cada rota;
 * 		-> Os locais são escolhidos pela menor distância para o depósito (respeitando a capacidade).
 * 2º) Para cada rota, enquanto a carga for menor do que a capacidade, inclui um novo local com menor soma das
 *     distâncias entre o último e o penúltimo local visitado, e que mantenha a carga dentro da capacidade;
 * 3º) Se sobrar local não visitado, acrescenta uma rota e faz, para esta, o mesmo processo descrito nos passos
 *     anteriores.
 */
int** CVRPsolver::gerarSolucaoGulosa() {
	int quantidadeCandidatos = 0;
	int* candidatos = new int[numLocais];
	int quantidadeAlocados = 1; // depósito

	bool* alocados = new bool[numLocais];
	for (int i = 0; i < numLocais; i++)
		alocados[i] = false;
	alocados[0] = true; // depósito

	int** solucao = gerarSolucaoVazia();

	// 1º)
	for (int r = 0; r < numMinDeCaminhoes; r++) {
		int local1 = vizinhosPorMenorDistancia[0][quantidadeAlocados++];
		int local2 = vizinhosPorMenorDistancia[0][quantidadeAlocados++];
		solucao[r][CVRPsolverConstantes::IND_NUM_LOCAIS] = 2;
		solucao[r][CVRPsolverConstantes::IND_CARGA] = demandas[local1] + demandas[local2];
		solucao[r][CVRPsolverConstantes::IND_INICIO_LOCAIS + 0] = local1;
		alocados[local1] = true;
		solucao[r][CVRPsolverConstantes::IND_INICIO_LOCAIS + 1] = local2;
		alocados[local2] = true;
	}

	int r = 0;
	int quantidadeRotas = numMinDeCaminhoes;
	int contagemRotasSemAlocacao = 0;
	while (quantidadeAlocados < numLocais) {
		// 2º)
		// monta lista de candidatos
		quantidadeCandidatos = 0;
		int ind = 0;
		for (int i = 0; i < numLocais; i++) {
			if (alocados[i] || solucao[r][CVRPsolverConstantes::IND_CARGA] + demandas[i] > capacidade)
				continue;
			candidatos[ind++] = i;
			quantidadeCandidatos++;
		}
		if (quantidadeCandidatos > 0) {
			// ordenar lista de candidatos por soma das distâncias para o penúltimo e para o último locais visitados
			int quantLocais = solucao[r][CVRPsolverConstantes::IND_NUM_LOCAIS];
			int ultimoLocal = solucao[r][CVRPsolverConstantes::IND_INICIO_LOCAIS + quantLocais - 1];
			int penultimoLocal = solucao[r][CVRPsolverConstantes::IND_INICIO_LOCAIS + quantLocais - 2];
			ordenarCandidatosPorDistancias(candidatos, quantidadeCandidatos, ultimoLocal, penultimoLocal);
			// escolhe primeiro da lista e o aloca na rota r
			int local = candidatos[0];
			solucao[r][CVRPsolverConstantes::IND_CARGA] =
					solucao[r][CVRPsolverConstantes::IND_CARGA] + demandas[local];
			solucao[r][CVRPsolverConstantes::IND_INICIO_LOCAIS + quantLocais] =
					solucao[r][CVRPsolverConstantes::IND_INICIO_LOCAIS + quantLocais - 1];
			solucao[r][CVRPsolverConstantes::IND_INICIO_LOCAIS + quantLocais - 1] = local;
			solucao[r][CVRPsolverConstantes::IND_NUM_LOCAIS] = quantLocais + 1;
			alocados[local] = true;
			quantidadeAlocados++;

			contagemRotasSemAlocacao = 0;
		} else {
			contagemRotasSemAlocacao++;
		}

		// passa para a próxima rota
		if (contagemRotasSemAlocacao < quantidadeRotas) {
			r = (r + 1) % quantidadeRotas;
		} else if (quantidadeAlocados < numLocais) {
			// 3º)
			quantidadeRotas++;
			r = quantidadeRotas - 1;
			// insere dois locais na rota criada (é claro, se houver dois locais não alocados)
			int inseridos = 0;
			for (int i = 1; i < numLocais; i++) {
				int local = vizinhosPorMenorDistancia[0][i];
				if (alocados[local])
					continue;
				solucao[r][CVRPsolverConstantes::IND_NUM_LOCAIS] = solucao[r][CVRPsolverConstantes::IND_NUM_LOCAIS] + 1;
				solucao[r][CVRPsolverConstantes::IND_CARGA] = solucao[r][CVRPsolverConstantes::IND_CARGA] + demandas[local];
				solucao[r][CVRPsolverConstantes::IND_INICIO_LOCAIS + inseridos] = local;
				alocados[local] = true;
				quantidadeAlocados++;
				inseridos++;
				if (inseridos == 2)
					break;
			}
		}
	}
	// calcular distâncias percorridas em cada rota
	for (int r = 0; r < quantidadeRotas; r++) {
		solucao[r][CVRPsolverConstantes::IND_DIST_PERCORRIDA] = calcularCustoSolucaoPorCaminhao(solucao, r);
	}

	return solucao;
}

int** CVRPsolver::gerarSolucaoGulosaAleatoria(double alfa) {
	return NULL;
}

void CVRPsolver::ordenarCandidatosPorDistancias(int* candidatos, int quantidadeCandidatos,
		int ultimoLocal, int penultimoLocal) {
	// aplicar ordenação por seleção
	for (int i = 0; i < quantidadeCandidatos; i++) {
		int indMenor = i;
		int localI = candidatos[i];
		int menorSomaDeDistancias = distancias[penultimoLocal][localI] + distancias[localI][ultimoLocal];
		for (int j = i + 1; j < quantidadeCandidatos; j++) {
			int localJ = candidatos[j];
			int somaDeDistancias = distancias[penultimoLocal][localJ] + distancias[localJ][ultimoLocal];
			if (somaDeDistancias < menorSomaDeDistancias) {
				indMenor = j;
				menorSomaDeDistancias = somaDeDistancias;
			}
		}
		if (indMenor != i) { // trocar posições
			int t = candidatos[i];
			candidatos[i] = candidatos[indMenor];
			candidatos[indMenor] = t;
		}
	}
}

void CVRPsolver::executarInsercaoEmNovaRota(int** rotaOriginal, int custoRotaOriginal, double custoAumentadoRotaOriginal,
		int** rotaAlterada, int* custoRotaAlterada, double* custoAumentadoRotaAlterada, int caminhao1, int indLocal1,
		int caminhao2) {
	int local = rotaOriginal[caminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1];

	for (int indCaminhao = 0; indCaminhao < this->numLocais; indCaminhao++) {
		int quantidadeLocais = rotaOriginal[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS];
		if (quantidadeLocais == 0) {
			rotaAlterada[indCaminhao][CVRPsolverConstantes::IND_NUM_LOCAIS] = 0;
			break;
		}

		memcpy(rotaAlterada[indCaminhao], rotaOriginal[indCaminhao], (quantidadeLocais + 3) * sizeof(int));
		if (rotaAlterada[indCaminhao][0] != rotaOriginal[indCaminhao][0])
			std::printf("Erro.\n");
	}

	int antecessor = indLocal1 == 0 ? 0 : rotaOriginal[caminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1 - 1];
	int sucessor = indLocal1 == rotaOriginal[caminhao1][CVRPsolverConstantes::IND_NUM_LOCAIS] - 1 ? 0 :
			rotaOriginal[caminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + indLocal1 + 1];

	// atualizar custo
	int custoArestasRemovidasLocalRemocao = this->distancias[antecessor][local] + this->distancias[local][sucessor];
	int custoArestaIncluidaLocalRemocao = this->distancias[antecessor][sucessor];
	double custoAumentadoArestasRemovidasLocalRemocao = custoArestasRemovidasLocalRemocao + this->lambda *
			(this->penalidades[antecessor][local] + this->penalidades[local][sucessor]);
	double custoAumentadoArestaIncluidaLocalRemocao = custoArestaIncluidaLocalRemocao + this->lambda *
			this->penalidades[antecessor][sucessor];

	rotaAlterada[caminhao1][CVRPsolverConstantes::IND_DIST_PERCORRIDA] += custoArestaIncluidaLocalRemocao -
			custoArestasRemovidasLocalRemocao;

	int custoArestasIncluidasLocalInsercao = 2 * this->distancias[0][local];
	double custoAumentadoArestasIncluidasLocalInsercao = custoArestasIncluidasLocalInsercao + this->lambda *
			this->penalidades[0][local];

	rotaAlterada[caminhao2][CVRPsolverConstantes::IND_DIST_PERCORRIDA] = custoArestasIncluidasLocalInsercao;

	// remover local da rota original
	int quantidadeLocais = rotaOriginal[caminhao1][CVRPsolverConstantes::IND_NUM_LOCAIS];
	for (int i = indLocal1 + 1; i < quantidadeLocais; i++)
		rotaAlterada[caminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + i - 1] =
				rotaOriginal[caminhao1][CVRPsolverConstantes::IND_INICIO_LOCAIS + i];
	(rotaAlterada[caminhao1][CVRPsolverConstantes::IND_NUM_LOCAIS])--;

	// incluir local na nova rota
	rotaAlterada[caminhao2][CVRPsolverConstantes::IND_INICIO_LOCAIS] = local;
	rotaAlterada[caminhao2][CVRPsolverConstantes::IND_NUM_LOCAIS] = 1;

	// ajustar cargas
	rotaAlterada[caminhao1][CVRPsolverConstantes::IND_CARGA] = rotaAlterada[caminhao1][CVRPsolverConstantes::IND_CARGA] -
			demandas[local];
	rotaAlterada[caminhao2][CVRPsolverConstantes::IND_CARGA] = demandas[local];

	// calcula custo da rota alterada

	*custoRotaAlterada = custoRotaOriginal - calcularCustoSolucaoPorCaminhao(rotaOriginal, caminhao1) +
			calcularCustoSolucaoPorCaminhao(rotaAlterada, caminhao1) +
			calcularCustoSolucaoPorCaminhao(rotaAlterada, caminhao2);
	*custoAumentadoRotaAlterada = custoAumentadoRotaOriginal -
			calcularCustoSolucaoPorCaminhao(rotaOriginal, caminhao1) -
			calcularTermoRegularizacaoSolucaoPorCaminhao(rotaOriginal, caminhao1) +
			calcularCustoSolucaoPorCaminhao(rotaAlterada, caminhao1) +
			calcularTermoRegularizacaoSolucaoPorCaminhao(rotaAlterada, caminhao1) +
			calcularCustoSolucaoPorCaminhao(rotaAlterada, caminhao2) +
			calcularTermoRegularizacaoSolucaoPorCaminhao(rotaAlterada, caminhao2);

	if (arredondar(*custoAumentadoRotaAlterada, 4) < arredondar(*custoRotaAlterada, 4))
		printf("erro");
}

bool CVRPsolver::verificarCapacidades(int** rota) {
	for (int c = 0; c < this->numLocais; c++) {
		int numLocaisCaminhao = rota[c][CVRPsolverConstantes::IND_NUM_LOCAIS];
		if (numLocaisCaminhao == 0)
			break;
		int carga = 0;
		for (int l = 0; l < numLocaisCaminhao; l++) {
			int local = rota[c][CVRPsolverConstantes::IND_INICIO_LOCAIS + l];
			carga += this->demandas[local];
		}
		if (carga > this->capacidade)
			return false;
	}
	return true;
}

int CVRPsolver::obterNumCaminhoesSolucao(int** solucao) {
	int numCaminhoes = 0;
	for (int i = 0; i < this->numLocais; i++)
		if (solucao[i][CVRPsolverConstantes::IND_NUM_LOCAIS] > 0)
			numCaminhoes++;
		else
			break;
	return numCaminhoes;
}

void CVRPsolver::iniciarMatrizDeVizinhosPorMenorDistancia() {
	vizinhosPorMenorDistancia = new int*[numLocais];
	bool* localUsado = new bool[numLocais];
	for (int local = 0; local < numLocais; local++) {
		vizinhosPorMenorDistancia[local] = new int[numLocais];
		vizinhosPorMenorDistancia[local][0] = local; // o mais próximo é o próprio local

		// reinicia o vetor de locais usados, para utilizar no processamento seguinte
		for (int l = 0; l < numLocais; l++)
			localUsado[l] = l == local;

		// encontra os locais com menores distâncias para o local atual
		for (int l = 1; l < numLocais; l++) {
			int menor = INT_MAX;
			int localMenorDistancia = -1;
			for (int vizinho = 0; vizinho < numLocais; vizinho++) {
				if (localUsado[vizinho])
					continue;
				if (distancias[local][vizinho] < menor) {
					menor = distancias[local][vizinho];
					localMenorDistancia = vizinho;
				}
			}
			vizinhosPorMenorDistancia[local][l] = localMenorDistancia;
			localUsado[localMenorDistancia] = true;
		}
	}
}

int* CVRPsolver::obterVetorCustoMelhorSolucao() {
	return this->custoMelhorSolucao;
}
