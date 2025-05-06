package fr.pir;

import org.springframework.boot.CommandLineRunner;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

import fr.phenix333.logger.MyLogger;

@SpringBootApplication
public class TTSApplication implements CommandLineRunner {

	private static final MyLogger L = MyLogger.create(TTSApplication.class);

	public static void main(String[] args) {
		L.function("");

		SpringApplication.run(TTSApplication.class, args);
	}

	@Override
	public void run(String... args) throws Exception {
		L.function("Lancement du code principal");

		//
	}

}