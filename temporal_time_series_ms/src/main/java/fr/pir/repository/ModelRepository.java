package fr.pir.repository;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import fr.pir.model.Model;

@Repository
public interface ModelRepository extends JpaRepository<Model, Long> {

	Model findModelByName(String name);

}
