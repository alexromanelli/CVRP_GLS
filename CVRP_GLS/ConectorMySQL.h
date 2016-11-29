/*
 * ConectorMySQL.h
 *
 *  Created on: 25/10/2016
 *      Author: romanelli
 */

#ifndef CONECTORMYSQL_H_
#define CONECTORMYSQL_H_

#include "mysql_connection.h"

#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>

class ConectorMySQL {
private:
	sql::Connection* con;
public:
	ConectorMySQL();
	virtual ~ConectorMySQL();

	int obterUltimoId();

	bool inserirTeste(std::string nomeInstancia, double lambda, int iteracoes, std::string opcaoBuscaLocal,
			std::string opcaoAprimoramento, int iteracoesAteReiniciarPenalidades, bool penalidadesSimetricas);
	bool buscarProximoTestePendente(std::string* nomeInstancia, double* lambda, int* iteracoes,
			std::string* opcaoBuscaLocal, std::string* opcaoAprimoramento, int* iteracoesAteReiniciarPenalidades,
			bool* penalidadesSimetricas);
	bool registrarTesteExecutado(std::string nomeInstancia, double lambda, int iteracoes,
			std::string opcaoBuscaLocal, std::string opcaoAprimoramento, int iteracoesAteReiniciarPenalidades,
			bool penalidadesSimetricas, int idResultado);

	int inserirResultado(std::string nomeInstancia, int capacidade, int quantidadeLocais,
			int custo, int quantidadeCaminhoes, double parametroLambda, double lambda, int iteracoes,
			std::string opcaoBuscaLocal, std::string opcaoAprimoramento, int iteracoesAteReiniciarPenalidades,
			bool penalidadesSimetricas, int tempo, int quantidadeSolucoesAvaliadas, int** solucao,
			int* demandas, double** coordenadas);

//	bool inserirProgressoExecucao(int idResultado, int numAgrupamentos, int numIteracoes,
//			int* custoMelhorOtimoLocal,	int* custoPiorOtimoLocal, double* custoMedioOtimoLocal,
//			double* desvpadCustoOtimoLocal,	double* custoAumentadoMelhorOtimoLocal,
//			double* custoAumentadoPiorOtimoLocal, double* custoAumentadoMedioOtimoLocal,
//			double* desvpadCustoAumentadoOtimoLocal);
	bool inserirProgressoExecucao(int idResultado, int numAgrupamentos, int numIteracoes,
			int* custoMelhorOtimoLocal);

};

#endif /* CONECTORMYSQL_H_ */
