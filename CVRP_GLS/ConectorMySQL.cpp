/*
 * ConectorMySQL.cpp
 *
 *  Created on: 25/10/2016
 *      Author: romanelli
 */

#include "ConectorMySQL.h"
#include "CVRPsolver.h"

#include "mysql_driver.h"

#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

ConectorMySQL::ConectorMySQL() {
	try {
		/* Create a connection */
		sql::mysql::MySQL_Driver* driver = sql::mysql::get_mysql_driver_instance();
		con = driver->connect("tcp://127.0.0.1:3306", "root", "romanelli");
		/* Connect to the MySQL test database */
		con->setSchema("cvrp");
	} catch (sql::SQLException &e) {
		std::cout << "# ERR: SQLException in " << __FILE__;
		std::cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << std::endl;
		std::cout << "# ERR: " << e.what();
		std::cout << " (MySQL error code: " << e.getErrorCode();
		std::cout << ", SQLState: " << e.getSQLState() << " )" << std::endl;
	}
}

ConectorMySQL::~ConectorMySQL() {
	delete con;
}

bool ConectorMySQL::inserirTeste(std::string nomeInstancia, double lambda, int iteracoes, std::string opcaoBuscaLocal,
		std::string opcaoAprimoramento, int iteracoesAteReiniciarPenalidades, bool penalidadesSimetricas) {
	bool result = false;

	try {
		sql::PreparedStatement* prepStmt =
				con->prepareStatement("insert into teste (instancia, lambda, iteracoes, opcao_busca_local, "
						"opcao_aprimoramento, iteracoes_ate_reiniciar_penalidades, penalidades_simetricas) "
						"values (?, ?, ?, ?, ?, ?, ?)");
		prepStmt->setString(1, nomeInstancia);
		prepStmt->setDouble(2, lambda);
		prepStmt->setInt(3, iteracoes);
		prepStmt->setString(4, opcaoBuscaLocal);
		prepStmt->setString(5, opcaoAprimoramento);
		prepStmt->setInt(6, iteracoesAteReiniciarPenalidades);
		prepStmt->setString(7, std::string((penalidadesSimetricas ? "s" : "n")));

		result = prepStmt->executeUpdate() > 0;

		delete prepStmt;

	} catch (sql::SQLException &e) {
		std::cout << "# ERR: SQLException in " << __FILE__;
		std::cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << std::endl;
		std::cout << "# ERR: " << e.what();
		std::cout << " (MySQL error code: " << e.getErrorCode();
		std::cout << ", SQLState: " << e.getSQLState() << " )" << std::endl;
	}

	return result;
}

bool ConectorMySQL::buscarProximoTestePendente(std::string* nomeInstancia, double* lambda, int* iteracoes,
		std::string* opcaoBuscaLocal, std::string* opcaoAprimoramento, int* iteracoesAteReiniciarPenalidades,
		bool* penalidadesSimetricas) {
	bool result = false;

	try {
		sql::Statement* stmt = con->createStatement();
		sql::ResultSet* res = stmt->executeQuery("" \
				"   select instancia, lambda, iteracoes, opcao_busca_local, "
				"          opcao_aprimoramento, iteracoes_ate_reiniciar_penalidades, "
				"          penalidades_simetricas " \
				"     from teste " \
				"    where executado = 'n' " \
				" order by data_cadastro asc " \
				"    limit 1");
		if (res->next()) {
			nomeInstancia->assign(res->getString(1).c_str());
			*lambda = res->getDouble(2);
			*iteracoes = res->getInt(3);
			*opcaoBuscaLocal = res->getString(4);
			*opcaoAprimoramento = res->getString(5);
			*iteracoesAteReiniciarPenalidades = res->getInt(6);
			*penalidadesSimetricas = res->getString(7)[0] == 's';
			result = true;
		}
		delete res;
		delete stmt;

	} catch (sql::SQLException &e) {
		std::cout << "# ERR: SQLException in " << __FILE__;
		std::cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << std::endl;
		std::cout << "# ERR: " << e.what();
		std::cout << " (MySQL error code: " << e.getErrorCode();
		std::cout << ", SQLState: " << e.getSQLState() << " )" << std::endl;
	}

	return result;
}

bool ConectorMySQL::registrarTesteExecutado(std::string nomeInstancia, double lambda, int iteracoes,
		std::string opcaoBuscaLocal, std::string opcaoAprimoramento, int iteracoesAteReiniciarPenalidades,
		bool penalidadesSimetricas, int idResultado) {
	bool result = false;

	try {
		sql::PreparedStatement* prepStmt =
				con->prepareStatement("update teste set executado = 's', "
						" resultado = ? where " \
						" instancia = ? and lambda = ? and iteracoes = ? and "
						" opcao_busca_local = ? and opcao_aprimoramento = ? and "
						" iteracoes_ate_reiniciar_penalidades = ? and "
						" penalidades_simetricas = ?");
		prepStmt->setInt(1, idResultado);
		prepStmt->setString(2, nomeInstancia);
		prepStmt->setDouble(3, lambda);
		prepStmt->setInt(4, iteracoes);
		prepStmt->setString(5, opcaoBuscaLocal);
		prepStmt->setString(6, opcaoAprimoramento);
		prepStmt->setInt(7, iteracoesAteReiniciarPenalidades);
		prepStmt->setString(8, (penalidadesSimetricas ? "s" : "n"));

		result = prepStmt->executeUpdate() > 0;

		delete prepStmt;

	} catch (sql::SQLException &e) {
		std::cout << "# ERR: SQLException in " << __FILE__;
		std::cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << std::endl;
		std::cout << "# ERR: " << e.what();
		std::cout << " (MySQL error code: " << e.getErrorCode();
		std::cout << ", SQLState: " << e.getSQLState() << " )" << std::endl;
	}

	return result;
}

int ConectorMySQL::obterUltimoId() {
	int ultimoId = -1;

	try {
		sql::Statement* stmt = con->createStatement();
		sql::ResultSet* res = stmt->executeQuery("select LAST_INSERT_ID()");
		if (res->next()) {
			ultimoId = res->getInt(1);
		}
		delete res;
		delete stmt;

	} catch (sql::SQLException &e) {
		std::cout << "# ERR: SQLException in " << __FILE__;
		std::cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << std::endl;
		std::cout << "# ERR: " << e.what();
		std::cout << " (MySQL error code: " << e.getErrorCode();
		std::cout << ", SQLState: " << e.getSQLState() << " )" << std::endl;
	}

	return ultimoId;
}

int ConectorMySQL::inserirResultado(std::string nomeInstancia, int capacidade, int quantidadeLocais,
		int custo, int quantidadeCaminhoes, double parametroLambda, double lambda, int iteracoes,
		std::string opcaoBuscaLocal, std::string opcaoAprimoramento, int iteracoesAteReiniciarPenalidades,
		bool penalidadesSimetricas, int tempo, int quantidadeSolucoesAvaliadas, int** solucao,
		int* demandas, double** coordenadas) {
	int idResultado = -1;

	try {
		// inserir registro de resultado
		sql::PreparedStatement* prepStmt =
				con->prepareStatement("insert into resultado " \
						" (instancia, capacidade, quantidade_locais, custo, quantidade_caminhoes, " \
						"  parametro_lambda, lambda, iteracoes, opcao_busca_local, opcao_aprimoramento, "
						"  iteracoes_ate_reiniciar_penalidades, penalidades_simetricas, tempo, "
						"  quantidade_solucoes_avaliadas) values " \
						" (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
		prepStmt->setString(1, nomeInstancia);
		prepStmt->setInt(2, capacidade);
		prepStmt->setInt(3, quantidadeLocais);
		prepStmt->setInt(4, custo);
		prepStmt->setInt(5, quantidadeCaminhoes);
		prepStmt->setDouble(6, parametroLambda);
		prepStmt->setDouble(7, lambda);
		prepStmt->setInt(8, iteracoes);
		prepStmt->setString(9, opcaoBuscaLocal);
		prepStmt->setString(10, opcaoAprimoramento);
		prepStmt->setInt(11, iteracoesAteReiniciarPenalidades);
		prepStmt->setString(12, penalidadesSimetricas ? "s" : "n");
		prepStmt->setInt(13, tempo);
		prepStmt->setInt(14, quantidadeSolucoesAvaliadas);

		bool sucesso = prepStmt->executeUpdate() > 0;

		delete prepStmt;

		if (sucesso) {
			// obter o id do registro de resultado
			idResultado = obterUltimoId();
			if (idResultado == -1)
				return false;

			// para cada caminhão, fazer o seu registro
			for (int caminhao = 0; caminhao < quantidadeCaminhoes; caminhao++) {
				// inserir registro de rota do caminhão
				prepStmt = con->prepareStatement("insert into rota_caminhao " \
								" (resultado, numero_caminhao, distancia_percurso, carga) values " \
								" (?, ?, ?, ?)");
				prepStmt->setInt(1, idResultado);
				prepStmt->setInt(2, caminhao + 1);
				prepStmt->setInt(3, solucao[caminhao][CVRPsolverConstantes::IND_DIST_PERCORRIDA]);
				prepStmt->setInt(4, solucao[caminhao][CVRPsolverConstantes::IND_CARGA]);

				sucesso = prepStmt->executeUpdate() > 0;

				delete prepStmt;

				if (sucesso) {
					// obter o id do registro de rota
					int idRota = obterUltimoId();
					if (idRota == -1)
						return false;

					// inserir registros de itens da rota

					// preparar comando sql
					std::string comandoSql = "insert into item_rota_caminhao " \
							" (rota_caminhao, sequencial, local, demanda, coord_x, coord_y) values ";
					for (int itemRota = 0; itemRota < solucao[caminhao][CVRPsolverConstantes::IND_NUM_LOCAIS]; itemRota++) {
						if (itemRota > 0)
							comandoSql += ",";
						comandoSql += " (?, ?, ?, ?, ?, ?)";
					}

					prepStmt = con->prepareStatement(comandoSql);

					// inserir parâmetros
					for (int itemRota = 0; itemRota < solucao[caminhao][CVRPsolverConstantes::IND_NUM_LOCAIS]; itemRota++) {
						int local = solucao[caminhao][CVRPsolverConstantes::IND_INICIO_LOCAIS + itemRota];
						int demanda = demandas[local];
						double coordX = coordenadas[local][0];
						double coordY = coordenadas[local][1];

						prepStmt->setInt(6 * itemRota + 1, idRota);
						prepStmt->setInt(6 * itemRota + 2, itemRota + 1);
						prepStmt->setInt(6 * itemRota + 3, local);
						prepStmt->setInt(6 * itemRota + 4, demanda);
						prepStmt->setDouble(6 * itemRota + 5, coordX);
						prepStmt->setDouble(6 * itemRota + 6, coordY);
					}

					sucesso = prepStmt->executeUpdate() > 0;
					delete prepStmt;
				}
			}
		}

	} catch (sql::SQLException &e) {
		std::cout << "# ERR: SQLException in " << __FILE__;
		std::cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << std::endl;
		std::cout << "# ERR: " << e.what();
		std::cout << " (MySQL error code: " << e.getErrorCode();
		std::cout << ", SQLState: " << e.getSQLState() << " )" << std::endl;
	}

	return idResultado;
}

bool ConectorMySQL::inserirProgressoExecucao(int idResultado, int numAgrupamentos,
		int numIteracoes, int* custoMelhorOtimoLocal) {
	bool result = false;

	try {
		// inserir registro de resultado
		std::string comandoSql = "insert into progresso_execucao_por_intervalo_de_iteracoes " \
				" (resultado, iteracao_inicial, iteracao_final, custo_melhor_solucao) values ";

		for (int i = 0; i < numAgrupamentos; i++) {
			if (i > 0)
				comandoSql += ", ";
			comandoSql += "(?, ?, ?, ?)";
		}

		sql::PreparedStatement* prepStmt = con->prepareStatement(comandoSql);
		int iteracoesPorAgrupamento = numIteracoes / numAgrupamentos;
		for (int i = 0; i < numAgrupamentos; i++) {
			int iteracaoInicial = i * iteracoesPorAgrupamento + 1;
			int iteracaoFinal = iteracaoInicial + iteracoesPorAgrupamento - 1;
			prepStmt->setInt(i * 4 + 1, idResultado);
			prepStmt->setInt(i * 4 + 2, iteracaoInicial);
			prepStmt->setInt(i * 4 + 3, iteracaoFinal);
			prepStmt->setInt(i * 4 + 4, custoMelhorOtimoLocal[i]);
		}

		result = prepStmt->executeUpdate() > 0;

		delete prepStmt;

	} catch (sql::SQLException &e) {
		std::cout << "# ERR: SQLException in " << __FILE__;
		std::cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << std::endl;
		std::cout << "# ERR: " << e.what();
		std::cout << " (MySQL error code: " << e.getErrorCode();
		std::cout << ", SQLState: " << e.getSQLState() << " )" << std::endl;
	}

	return result;
}
