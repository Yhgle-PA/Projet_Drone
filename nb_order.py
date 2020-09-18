def number_order(dict_setup, t):
    for i in range(0, len(dict_setup["proba_h_order"]), 3):
        if t >= dict_setup["proba_h_order"][i]*3600 and t < dict_setup["proba_h_order"][i+1]*3600:
            nb_order = dict_setup["proba_h_order"][i+2] * dict_setup["nb_order_day"]/(dict_setup["proba_h_order"][i+1]*3600-dict_setup["proba_h_order"][i]*3600)*dict_setup["dt"]
            return int(nb_order)


def check_proba_order(dict_setup):
    if dict_setup["proba_h_order"][0] != 0:
        raise Exception('The proba_h_order should begin with the hour 0')
    if dict_setup["proba_h_order"][-2] != 24:
        raise Exception('The proba_h_order should end with the hour 24')

    for i in range(0, len(dict_setup["proba_h_order"]), 3):
        if i != 0 and dict_setup["proba_h_order"][i] != dict_setup["proba_h_order"][i-2]:
            raise Exception('The proba_h_order have a gap in the hours')

    if sum(dict_setup["proba_h_order"][i] for i in range(len(dict_setup["proba_h_order"])) if i%3 == 2) != 1:
        raise Exception('The sum of probabilities in the proba_h_order should be 1')
