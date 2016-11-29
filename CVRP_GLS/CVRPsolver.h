/*
 * CVRPsolver.h
 *
 *  Created on: 12/10/2016
 *      Author: romanelli
 */

#ifndef CVRPSOLVER_H_
#define CVRPSOLVER_H_

#include <string>

namespace CVRPsolverConstantes {
	const int IND_NUM_LOCAIS = 0;
	const int IND_CARGA = 1;
	const int IND_DIST_PERCORRIDA = 2;
	const int IND_INICIO_LOCAIS = 3;

	const int OPCAO_PRIMEIRO_APRIMORANTE = 0;
	const int OPCAO_MELHOR_APRIMORANTE = 1;

	const int OPCAO_BUSCA_LOCAL_CONVENCIONAL = 0;
	const int OPCAO_BUSCA_LOCAL_RAPIDA = 1;
}

class CVRPsolver {
private:
	std::string nomeInstancia;
	int numLocais;
	int indDeposito;
	int numMinDeCaminhoes;
	int capacidade;
	int** distancias;
	int distanciaMaxima;
	int** vizinhosPorMenorDistancia;
	int* demandas;
	double lambda;
	int** penalidades; // penalidade aplicada ao uso de aresta na solução
	bool penalidadesSimetricas; // se penalizar aresta i->j, penalizar também aresta j->i
	int numMaxIteracoes;
	int iteracoesAteReiniciarPenalidades;
	int opcaoAprimorante;
	int opcaoBuscaLocal;
	bool imprimirAcompanhamento;
	bool salvarDadosDeProgressoDaExecucao;

	// para o caso de salvar dados do progresso da execução
	int numAgrupamentosProgressoExecucao;
	int* custoMelhorSolucao;

	// para o caso de busca local rápida
	bool* vizinhancaAtivada;
	bool* vizinhancaTemSolucaoMelhor;

	long int quantidadeSolucoesAvaliadas;

	// campos usados na busca local
	int** vizinho;
	int** melhorVizinho;
	int** otimoLocal;

	// flag indicador de necessidade de alocar campos usados na busca local
	bool solucoesAuxiliaresNulas;

public:
	CVRPsolver(std::string ni, int nl, int id, int nmdc, int cap, int** dist, int* dem, double lambda, int dm, int nmi,
			int iarp, int oa, bool ia, bool ps, bool sddpde);
	virtual ~CVRPsolver();

	static double get_cpu_time();

	void setLambda(double lambda);
	double getLambda();
	int getNumLocais();

	void iniciarVetorDeAtivacaoDeVizinhanca();
	void iniciarMatrizDeVizinhosPorMenorDistancia();

	//void calcularValoresEstatisticosProgressoExecucao(int indiceDadosProgresso, int tamanhoAgrupamento);

	void ordenarCandidatosPorDistancias(int* candidatos, int quantidadeCandidatos,
			int ultimoLocal, int penultimoLocal);

	/**
	 * Solução representada com uma matriz de números inteiros n x (n + 3), sendo que:
	 * - Há uma linha para cada caminhão;
	 * - O primeiro valor da linha indica a quantidade M de cidades visitadas pelo caminhão;
	 * - O segundo valor da linha indica a carga C do caminhão (restrição: C <= capacidade);
	 * - O terceiro valor da linha indica a distância percorrida pelo caminhão
	 * - Os próximos M valores da linha indicam as cidades visitadas, na ordem em que são visitadas.
	 *
	 * Se o primeiro valor da linha for zero, significa que o caminhão não está em uso.
	 * Todos os caminhões em uso são agrupados nas primeiras linhas, antes de qualquer linha que
	 * represente algum caminhão fora de uso.
	 */
	int** resolver();
	int** gerarSolucaoVazia();
	void iniciarPenalidades();
	void zerarPenalidades();
	void ativarTodasAsVizinhancas();
	int** gerarSolucaoAleatoria();

	/**
	 * Gera uma solução seguindo o seguinte método heurístico:
	 * 1º) Com base no número mínimo de veículos, atribui dois locais visitados (inicial e final) a cada rota;
	 * 		-> Os locais são escolhidos pela menor distância para o depósito (respeitando a capacidade).
	 * 2º) Para cada rota, enquanto a carga for menor do que a capacidade, inclui um novo local com menor soma das
	 *     distâncias entre o último e o penúltimo local visitado, e que mantenha a carga dentro da capacidade;
	 * 3º) Se sobrar local não visitado, acrescenta uma rota e faz, para esta, o mesmo processo descrito nos passos
	 *     anteriores.
	 */
	int** gerarSolucaoGulosa();

	int** gerarSolucaoGulosaAleatoria(double alfa);
	void executar2Opt(int** rotaOriginal, int custoRotaOriginal, double custoAumentadoRotaOriginal,
			int** rotaAlterada, int* custoRotaAlterada, double* custoAumentadoRotaAlterada,
			int caminhao, int indLocal1, int indLocal2);
	bool executarTroca(int** rotaOriginal, int custoRotaOriginal, double custoAumentadoRotaOriginal,
			int** rotaAlterada, int* custoRotaAlterada, double* custoAumentadoRotaAlterada,
			int caminhao1, int indLocal1, int indCaminhao2, int indLocal2);
	bool executarInsercao(int** rotaOriginal, int custoRotaOriginal, double custoAumentadoRotaOriginal,
			int** rotaAlterada, int* custoRotaAlterada, double* custoAumentadoRotaAlterada,
			int caminhao1, int indLocal1, int caminhao2, int indLocal2);
	void executarInsercaoEmNovaRota(int** rotaOriginal, int custoRotaOriginal, double custoAumentadoRotaOriginal,
			int** rotaAlterada, int* custoRotaAlterada, double* custoAumentadoRotaAlterada,
			int caminhao1, int indLocal1, int caminhao2);
	int** localSearch(int** s0, int** melhorSolucaoEmG, long int* quantSolucoesAvaliadas, int* quantMelhorias,
			bool* melhoriaEmG, bool permitirNovasRotas);
	double calcularCustoSolucao(int** solucao, bool usarPenalidades, bool aplicarCustoCaminhoes);
	int calcularCustoSolucaoPorDistancias(int** solucao);
	double calcularCustoAumentadoSolucaoPorDistancias(int** solucao);
	int calcularCustoSolucaoPorCaminhao(int** solucao, int indCaminhao);
	double calcularTermoRegularizacaoSolucaoPorCaminhao(int** solucao, int indCaminhao);
	double expressaoUtilidade(int origem, int destino);
	int** guidedLocalSearch(std::string nomeMetodoBuscaLocal, std::string opcaoAprimoramento);
	void copiarRota(int** rotaOriginal, int** rotaCopia);
	int obterDistancia(int local1, int local2);
	int obterNumCaminhoesSolucao(int** solucao);
	int obterQuantidadeSolucoesAvaliadas();

	int* obterVetorCustoMelhorSolucao();

	bool verificarCapacidades(int** rota);

	int getNumAgrupamentosProgressoExecucao() const {
		return numAgrupamentosProgressoExecucao;
	}
};

#endif /* CVRPSOLVER_H_ */
